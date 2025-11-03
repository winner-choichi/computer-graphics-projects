////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>

#include <GL/glew.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "cvec.h"
#include "matrix4.h"
#include "geometrymaker.h"
#include "ppm.h"
#include "glsupport.h"

using namespace std; // for string, vector, iostream, and other standard C++ stuff

// G L O B A L S ///////////////////////////////////////////////////

// --------- IMPORTANT --------------------------------------------------------
// Before you start working on this assignment, set the following variable
// properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// OpenGL 3.x+ with GLSL 1.3.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to
// use GLSL 1.3. Make sure that your machine supports the version of GLSL you
// are using. In particular, on Mac OS X currently there is no way of using
// OpenGL 3.x with GLSL 1.3 when GLUT is used.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get
// loaded
// ----------------------------------------------------------------------------
static const bool g_Gl2Compatible = true;

static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;  // near plane
static const float g_frustFar = -50.0;  // far plane
static const float g_groundY = -2.0;    // y coordinate of the ground
static const float g_groundSize = 10.0; // half the ground length

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false; // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
// 이거 왜 0(solid)로 설정하면 cout이 안보이지?
static int g_activeShader = 0;

// ============================================================================
// Define your own global variables here if needed.
// ============================================================================
// static int g_newVariable = 0;
static int g_numCube = 2;
static char viewpoint = 's';
static char ctrlObj = 's';
static char wldSky = 'w';

static Matrix4 auxilaryFrame, auxilaryT, auxilaryR;

// ============================================================================

struct ShaderState
{
  GlProgram program;

  // Handles to uniform variables
  GLint h_uLight, h_uLight2;
  GLint h_uProjMatrix;      // vertex shader에서 eye coord. projection하는 행렬
  GLint h_uModelViewMatrix; // vertex shader에서 object coord. to eye coord.하는 행렬
  GLint h_uNormalMatrix;    // normal vector to eye coord
  GLint h_uColor;

  // Handles to vertex attributes
  GLint h_aPosition;
  GLint h_aNormal;

  ShaderState(const char *vsfn, const char *fsfn)
  {
    readAndCompileShader(program, vsfn, fsfn);

    const GLuint h = program; // short hand

    // Retrieve handles to uniform variables
    h_uLight = safe_glGetUniformLocation(h, "uLight");
    h_uLight2 = safe_glGetUniformLocation(h, "uLight2");
    h_uProjMatrix = safe_glGetUniformLocation(h, "uProjMatrix");
    h_uModelViewMatrix = safe_glGetUniformLocation(h, "uModelViewMatrix");
    h_uNormalMatrix = safe_glGetUniformLocation(h, "uNormalMatrix");
    h_uColor = safe_glGetUniformLocation(h, "uColor");

    // Retrieve handles to vertex attributes
    h_aPosition = safe_glGetAttribLocation(h, "aPosition");
    h_aNormal = safe_glGetAttribLocation(h, "aNormal");

    if (!g_Gl2Compatible)
      glBindFragDataLocation(h, 0, "fragColor");
    checkGlErrors();
  }
};

static const int g_numShaders = 2;
static const char *const g_shaderFiles[g_numShaders][2] = {
    {"./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader"},
    {"./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader"}};
static const char *const g_shaderFilesGl2[g_numShaders][2] = {
    {"./shaders/basic-gl2.vshader", "./shaders/diffuse-gl2.fshader"},
    {"./shaders/basic-gl2.vshader", "./shaders/solid-gl2.fshader"}};
static vector<shared_ptr<ShaderState>> g_shaderStates; // our global shader states

// --------- Geometry

// Macro used to obtain relative offset of a field within a struct. draw할 때 glVertexAttribPointer에서 offset구할 때 씀.
#define FIELD_OFFSET(StructType, field) &(((StructType *)0)->field)

// A vertex with floating point position and normal
struct VertexPN
{
  Cvec3f p, n;

  VertexPN() {}
  VertexPN(float x, float y, float z,
           float nx, float ny, float nz)
      : p(x, y, z), n(nx, ny, nz)
  {
  }

  // Define copy constructor and assignment operator from GenericVertex so we can
  // use make* functions from geometrymaker.h
  VertexPN(const GenericVertex &v)
  {
    *this = v;
  }

  VertexPN &operator=(const GenericVertex &v)
  {
    p = v.pos;
    n = v.normal;
    return *this;
  }
};

struct Geometry
{
  GlBufferObject vbo, ibo;
  int vboLen, iboLen;

  Geometry(VertexPN *vtx, unsigned short *idx, int vboLen, int iboLen)
  {
    this->vboLen = vboLen;
    this->iboLen = iboLen;

    // Now create the VBO and IBO
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPN) * vboLen, vtx, GL_STATIC_DRAW); // gpu에 vtx배열을 복사해서 보냄(총 vboLen개)
    // 이제부터 gpu에 보낸 vtx 쓰고싶으면 vbo 바인드해서 쓰면 됨.
    // gpu에서 쓰는 이유는? draw 할 때 불러와 컴파일해둔 쉐이더 프로그램이랑 합쳐서 그림.

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * iboLen, idx, GL_STATIC_DRAW);
  }

  void draw(const ShaderState &curSS)
  {
    // Enable the attributes used by our shader
    safe_glEnableVertexAttribArray(curSS.h_aPosition);
    safe_glEnableVertexAttribArray(curSS.h_aNormal);

    // bind vbo
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    safe_glVertexAttribPointer(curSS.h_aPosition, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, p)); // vbo gpu에 저장할 때 VertexPN그대로 저장해서.
    safe_glVertexAttribPointer(curSS.h_aNormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, n));

    // bind ibo
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

    // draw!
    glDrawElements(GL_TRIANGLES, iboLen, GL_UNSIGNED_SHORT, 0); // ibo은 VertexPN아닌 그냥 배열이라 FIELD_OFFSET대신 0

    // Disable the attributes used by our shader
    safe_glDisableVertexAttribArray(curSS.h_aPosition);
    safe_glDisableVertexAttribArray(curSS.h_aNormal);
  }
};

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube;

// --------- Scene

static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0); // define two lights positions in world space
static Matrix4 g_skyRbt = Matrix4::makeTranslation(Cvec3(0.0, 0.25, 4.0));
// asst2-> 큐브 두 개 생성
static Matrix4 g_objectRbt[2] = {Matrix4::makeTranslation(Cvec3(-1, 0, 0)), Matrix4::makeTranslation(Cvec3(1, 0, 0))}; // left box, right box
static Cvec3f g_objectColors[2] = {Cvec3f(1, 0, 0), Cvec3f(0, 0, 1)};                                                  // left box color
///////////////// END OF G L O B A L S //////////////////////////////////////////////////

static void initGround()
{
  // A x-z plane at y = g_groundY of dimension [-g_groundSize, g_groundSize]^2
  VertexPN vtx[4] = {
      VertexPN(-g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
      VertexPN(-g_groundSize, g_groundY, g_groundSize, 0, 1, 0),
      VertexPN(g_groundSize, g_groundY, g_groundSize, 0, 1, 0),
      VertexPN(g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
  };
  unsigned short idx[] = {0, 1, 2, 0, 2, 3}; // CCW가 기본적으로 앞면되게 설정돼있음
  g_ground.reset(new Geometry(&vtx[0], &idx[0], 4, 6));
}

static void initCubes()
{
  int ibLen, vbLen;
  getCubeVbIbLen(vbLen, ibLen); // vertex buffer length

  // Temporary storage for cube geometry
  vector<VertexPN> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeCube(1, vtx.begin(), idx.begin());
  g_cube.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen)); // geometry 구조체 만들기
}

// takes a projection matrix and send to the the shaders
static void sendProjectionMatrix(const ShaderState &curSS, const Matrix4 &projMatrix)
{
  GLfloat glmatrix[16];
  projMatrix.writeToColumnMajorMatrix(glmatrix); // send projection matrix
  safe_glUniformMatrix4fv(curSS.h_uProjMatrix, glmatrix);
}

// takes MVM and its normal matrix to the shaders
static void sendModelViewNormalMatrix(const ShaderState &curSS, const Matrix4 &MVM, const Matrix4 &NMVM)
{
  GLfloat glmatrix[16];
  MVM.writeToColumnMajorMatrix(glmatrix); // send MVM
  safe_glUniformMatrix4fv(curSS.h_uModelViewMatrix, glmatrix);

  NMVM.writeToColumnMajorMatrix(glmatrix); // send NMVM
  safe_glUniformMatrix4fv(curSS.h_uNormalMatrix, glmatrix);
}

// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY()
{
  if (g_windowWidth >= g_windowHeight)
    g_frustFovY = g_frustMinFov;
  else
  {
    const double RAD_PER_DEG = 0.5 * CS175_PI / 180;
    g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
  }
}

static Matrix4 makeProjectionMatrix()
{
  return Matrix4::makeProjection(
      g_frustFovY, g_windowWidth / static_cast<double>(g_windowHeight),
      g_frustNear, g_frustFar);
}

static void drawStuff()
{
  // short hand for current shader state
  const ShaderState &curSS = *g_shaderStates[g_activeShader];

  // build & send proj. matrix to vshader
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(curSS, projmat);

  // use the skyRbt as the eyeRbt
  // asst2 -> v누르면 시점 전환
  // const Matrix4 eyeRbt = g_skyRbt;
  Matrix4 eyeRbt;
  switch (viewpoint)
  {
  case 's':
    eyeRbt = g_skyRbt;
    break;
  case 'l':
    eyeRbt = g_objectRbt[0];
    break;
  case 'r':
    eyeRbt = g_objectRbt[1];
    break;
  }
  const Matrix4 invEyeRbt = inv(eyeRbt);

  const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(g_light1, 1)); // g_light1 position in eye coordinates
  const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(g_light2, 1)); // g_light2 position in eye coordinates
  safe_glUniform3f(curSS.h_uLight, eyeLight1[0], eyeLight1[1], eyeLight1[2]);
  safe_glUniform3f(curSS.h_uLight2, eyeLight2[0], eyeLight2[1], eyeLight2[2]);

  // draw ground
  // ===========
  //
  const Matrix4 groundRbt = Matrix4(); // identity
  Matrix4 MVM = invEyeRbt * groundRbt;
  Matrix4 NMVM = normalMatrix(MVM);
  sendModelViewNormalMatrix(curSS, MVM, NMVM);
  safe_glUniform3f(curSS.h_uColor, 0.1, 0.95, 0.1); // set color
  g_ground->draw(curSS);

  // draw cubes
  // ==========
  for (int i = 0; i < g_numCube; i++)
  {
    MVM = invEyeRbt * g_objectRbt[i];
    NMVM = normalMatrix(MVM);
    sendModelViewNormalMatrix(curSS, MVM, NMVM);
    safe_glUniform3f(curSS.h_uColor, g_objectColors[i][0], g_objectColors[i][1], g_objectColors[i][2]);
    g_cube->draw(curSS);
  }
}

static void display()
{
  glUseProgram(g_shaderStates[g_activeShader]->program);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear framebuffer color&depth

  drawStuff();

  glutSwapBuffers(); // show the back buffer (where we rendered stuff)

  checkGlErrors();
}

static void reshape(const int w, const int h)
{
  g_windowWidth = w;
  g_windowHeight = h;
  glViewport(0, 0, w, h);
  cerr << "Size of window is now " << w << "x" << h << endl;
  updateFrustFovY();
  glutPostRedisplay();
}

static void makeAuxFrame()
{
  switch (viewpoint)
  {
  case 'l':
    auxilaryR = linFact(g_objectRbt[0]);
    break;
  case 'r':
    auxilaryR = linFact(g_objectRbt[1]);
    break;
  case 's':
    auxilaryR = linFact(g_skyRbt);
    break;
  }
  switch (ctrlObj)
  {
  case 'l':
    auxilaryT = transFact(g_objectRbt[0]);
    break;
  case 'r':
    auxilaryT = transFact(g_objectRbt[1]);
    break;
  case 's':
    if (wldSky == 's') // m에 따라 world-sky, sky-sky바꾸는거
    {
      auxilaryT = transFact(g_skyRbt);
    }
    else
    {
      auxilaryT = Matrix4();
    }
    break;
  }
  auxilaryFrame = auxilaryT * auxilaryR;
}

static void motion(const int x, const int y)
{
  const double dx = x - g_mouseClickX;
  const double dy = g_windowHeight - y - 1 - g_mouseClickY;

  Matrix4 m;
  if (g_mouseLClickButton && !g_mouseRClickButton)
  { // left button down?
    m = Matrix4::makeXRotation(-dy) * Matrix4::makeYRotation(dx);
  }
  else if (g_mouseRClickButton && !g_mouseLClickButton)
  { // right button down?
    m = Matrix4::makeTranslation(Cvec3(dx, dy, 0) * 0.01);
  }
  else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton))
  { // middle or (left and right) button down?
    m = Matrix4::makeTranslation(Cvec3(0, 0, -dy) * 0.01);
  }

  if (g_mouseClickDown)
  {
    makeAuxFrame();
    if (ctrlObj == viewpoint){
      if (viewpoint == 's' && wldSky == 'w') m = inv(m);
      else m = transFact(m)*inv(linFact(m));
    }
    if (ctrlObj == 's' && (viewpoint == 'l' || viewpoint == 'r'))
      m = Matrix4(); // cube가 sky 바꾸려 하면 변환 identity로 안바뀌게.
    switch (ctrlObj)
    {
    case 's':
      g_skyRbt = auxilaryFrame * m * inv(auxilaryFrame) * g_skyRbt; // o^t = w^t * (A * A^-1 * O) -> m wrt a
      break;
    case 'l':
      g_objectRbt[0] = auxilaryFrame * m * inv(auxilaryFrame) * g_objectRbt[0];
      break;
    case 'r':
      g_objectRbt[1] = auxilaryFrame * m * inv(auxilaryFrame) * g_objectRbt[1];
      break;
    }
    glutPostRedisplay(); // we always redraw if we changed the scene
  }

  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;
}

static void mouse(const int button, const int state, const int x, const int y)
{
  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1; // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

  g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
  g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
  g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

  g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
  g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
  g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

  g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;
}

static void keyboard(const unsigned char key, const int x, const int y)
{
  switch (key)
  {
  case 27:
    exit(0); // ESC
  case 'h':
    cout << " ============== H E L P ==============\n\n"
         << "h\t\thelp menu\n"
         << "s\t\tsave screenshot\n"
         << "f\t\tToggle flat shading on/off.\n"
         << "o\t\tCycle object to edit\n"
         << "v\t\tCycle view\n"
         << "drag left mouse to rotate\n"
         << endl;
    break;
  case 's':
    glFlush();
    writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
    break;
  case 'f':
    g_activeShader ^= 1;
    break;
  case 'v':
    // asst2-> v눌리면 시점 전환: sky > l cube > r cube > sky
    switch (viewpoint)
    {
    case 's':
      cout << "Active eye is Object 0" << endl;
      viewpoint = 'l';
      break;
    case 'l':
      cout << "Active eye is Object 1" << endl;
      viewpoint = 'r';
      break;
    case 'r':
      cout << "Active eye is Sky" << endl;
      viewpoint = 's';
      break;
    }
    break;
  case 'o':
    // asst2-> o눌리면 컨트롤 오브젝트 전환: sky > l cube > r cube > sky
    switch (ctrlObj)
    {
    case 's':
      cout << "Active object is Object 0" << endl;
      ctrlObj = 'l';
      break;
    case 'l':
      cout << "Active object is Object 1" << endl;
      ctrlObj = 'r';
      break;
    case 'r':
      cout << "Active object is Sky" << endl;
      ctrlObj = 's';
      break;
    }
    break;
  case 'm':
    if (wldSky == 's'){
      cout << "Editing sky eye w.r.t. world-sky frame" << endl;
      wldSky = 'w';
    } else {
      cout << "Editing sky eye w.r.t. sky-sky frame" << endl;
      wldSky = 's';
    }
    break;
  }
  glutPostRedisplay();
}

static void initGlutState(int argc, char *argv[])
{
  glutInit(&argc, argv);                                     // initialize Glut based on cmd-line args
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH); //  RGBA pixel channels and double buffering
  glutInitWindowSize(g_windowWidth, g_windowHeight);         // create a window
  glutCreateWindow("Assignment 2");                          // title the window

  glutDisplayFunc(display); // display rendering callback
  glutReshapeFunc(reshape); // window reshape callback
  glutMotionFunc(motion);   // mouse movement callback
  glutMouseFunc(mouse);     // mouse click callback
  glutKeyboardFunc(keyboard);
}

static void initGLState()
{
  glClearColor(128. / 255., 200. / 255., 255. / 255., 0.);
  glClearDepth(0.);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_GREATER);
  glReadBuffer(GL_BACK);
  if (!g_Gl2Compatible)
    glEnable(GL_FRAMEBUFFER_SRGB);
}

static void initShaders()
{
  g_shaderStates.resize(g_numShaders);
  for (int i = 0; i < g_numShaders; ++i)
  {
    if (g_Gl2Compatible)
      g_shaderStates[i].reset(new ShaderState(g_shaderFilesGl2[i][0], g_shaderFilesGl2[i][1]));
    else
      g_shaderStates[i].reset(new ShaderState(g_shaderFiles[i][0], g_shaderFiles[i][1]));
  }
}

static void initGeometry()
{
  initGround();
  initCubes();
}

int main(int argc, char *argv[])
{
  try
  {
    initGlutState(argc, argv);

    glewInit(); // load the OpenGL extensions

    cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3") << endl;
    if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
    else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");

    initGLState();
    initShaders();
    initGeometry();

    glutMainLoop();
    return 0;
  }
  catch (const runtime_error &e)
  {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}
