////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

// standard c++ libraries
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>

// OpenGL + GLEW/GLUT
#include <GL/glew.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

// Math & Geometry Utils
#include "cvec.h"
#include "matrix4.h"
#include "geometrymaker.h"
#include "quat.h"
#include "rigtform.h"

// UI & Interaction
#include "arcball.h"

// Rendering & Image Support
#include "ppm.h"
#include "glsupport.h"

// Scene Graph & Visitor Pattern
#include "asstcommon.h"
#include "scenegraph.h"
#include "drawer.h"
#include "picker.h"

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
const bool g_Gl2Compatible = true;

static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)
static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane

static const float g_groundY = -2.0;    // y coordinate of the ground
static const float g_groundSize = 10.0; // half the ground length

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false; // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;

// ============================================================================
// Define your own global variables here if needed.
// ============================================================================
static char viewpoint = 's';
static char ctrlObj = 's';
static char wldSky = 'w';
static int g_arcballScreenRadius = min(g_windowHeight, g_windowWidth) * 0.25;
static double g_arcballScale;
static Cvec3 g_arcUnitVec;
bool drawArc = false;
bool g_pickMode = false;
bool g_nothingPicked = true;

static RigTForm auxilaryFrame, auxilaryT, auxilaryR, eyeRbt;

static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode; // used later when you do picking

static const int PICKING_SHADER = 2; // index of the picking shader is g_shaerFiles
static const int g_numShaders = 3;
static const char *const g_shaderFiles[g_numShaders][2] = {
    {"./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader"},
    {"./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader"},
    {"./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"}};
static const char *const g_shaderFilesGl2[g_numShaders][2] = {
    {"./shaders/basic-gl2.vshader", "./shaders/diffuse-gl2.fshader"},
    {"./shaders/basic-gl2.vshader", "./shaders/solid-gl2.fshader"},
    {"./shaders/basic-gl2.vshader", "./shaders/pick-gl2.fshader"}};
static vector<shared_ptr<ShaderState>> g_shaderStates; // our global shader states

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

typedef SgGeometryShapeNode<Geometry> MyShapeNode;

static shared_ptr<Geometry> g_ground, g_cube, g_sphere; // Vertex buffer and index buffer associated with the ground and cube geometry

// ============================================================================
// Interface, Light Source
// ============================================================================
static const RigTForm g_light1(Cvec3(2.0, 3.0, 14.0)), g_light2(Cvec3(-2, -3.0, -5.0)); // define two lights positions in world space asst3

// asst3-> Arcball
static RigTForm g_arcballRbt;
static Cvec3f g_arcballColor = Cvec3f(0, 1, 0);

///////////////// END OF G L O B A L S //////////////////////////////////////////////////

// Create vertex/index data, Uploading them to GPU via a new Geometry object, Assign that object to the g_ground global variable
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

// Create vertex/index data, Uploading them to GPU via a new Geometry object, Assign that object to the g_cube global variable
static void initCubes()
{
  int ibLen, vbLen;
  getCubeVbIbLen(vbLen, ibLen); // vertex buffer length
  vector<VertexPN> vtx(vbLen);
  vector<unsigned short> idx(ibLen);
  makeCube(1, vtx.begin(), idx.begin());

  g_cube.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen)); // geometry 구조체 만들기
}

// Create vertex/index data, Uploading them to GPU via a new Geometry object, Assign that object to the g_sphere global variable
static void initSphere()
{
  int ibLen, vbLen, slices = 20, stacks = 10;
  getSphereVbIbLen(slices, stacks, vbLen, ibLen);
  vector<VertexPN> vtx(vbLen);
  vector<unsigned short> idx(ibLen);
  makeSphere(1, slices, stacks, vtx.begin(), idx.begin());

  g_sphere.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

// takes a projection matrix and send to the the shaders
static void sendProjectionMatrix(const ShaderState &curSS, const Matrix4 &projMatrix)
{
  GLfloat glmatrix[16];
  projMatrix.writeToColumnMajorMatrix(glmatrix); // send projection matrix
  safe_glUniformMatrix4fv(curSS.h_uProjMatrix, glmatrix);
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

// return new projection matrix with global settings
static Matrix4 makeProjectionMatrix()
{
  return Matrix4::makeProjection(
      g_frustFovY, g_windowWidth / static_cast<double>(g_windowHeight),
      g_frustNear, g_frustFar);
}

static void drawStuff(const ShaderState &curSS, bool picking)
{
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(curSS, projmat);

  // when user picked nothing, manipulating object follows viewpoint
  switch (viewpoint)
  {
  case 's':
    if (g_nothingPicked)
    {
      g_currentPickedRbtNode = NULL;
    }
    eyeRbt = getPathAccumRbt(g_world, g_skyNode);
    break;
  case 'l':
    if (g_nothingPicked)
    {
      g_currentPickedRbtNode = g_robot1Node;
    }
    eyeRbt = getPathAccumRbt(g_world, g_robot1Node);
    break;
  case 'r':
    if (g_nothingPicked)
    {
      g_currentPickedRbtNode = g_robot2Node;
    }
    eyeRbt = getPathAccumRbt(g_world, g_robot2Node);
    break;
  }
  const RigTForm invEyeRbt = inv(eyeRbt);

  // Set light source positions using unifrom variables
  const RigTForm eyeLight1 = RigTForm(invEyeRbt * g_light1); // g_light1 position in eye coordinates
  const RigTForm eyeLight2 = RigTForm(invEyeRbt * g_light2); // g_light2 position in eye coordinates
  safe_glUniform3f(curSS.h_uLight, eyeLight1.getTranslation()[0], eyeLight1.getTranslation()[1], eyeLight1.getTranslation()[2]);
  safe_glUniform3f(curSS.h_uLight2, eyeLight2.getTranslation()[0], eyeLight2.getTranslation()[1], eyeLight2.getTranslation()[2]);

  // Determine to draw arcball & compute z depth of controlled obect for arcball scaling
  float z;
  drawArc = false;
  if ((viewpoint == 's' && wldSky == 'w') && g_nothingPicked)
  {
    z = invEyeRbt.getTranslation()[2]; // world라서 O생략, t의 z성분만
    drawArc = true;
  }
  else if ((viewpoint == 's' && !g_nothingPicked) || (viewpoint == 'l' && g_currentPickedRbtNode != g_robot1Node) || (viewpoint == 'r' && g_currentPickedRbtNode != g_robot2Node))
  {
    z = (invEyeRbt * getPathAccumRbt(g_world, g_currentPickedRbtNode)).getTranslation()[2];
    drawArc = true;
  }

  if (z > CS175_EPS)
  {
    drawArc = false;
  }

  if (!picking)
  {
    Drawer drawer(invEyeRbt, curSS);
    g_world->accept(drawer);

    // draw Arcball
    if (g_nothingPicked && viewpoint == 's')
    {
      g_arcballRbt = RigTForm();
    }
    else
    {
      g_arcballRbt = getPathAccumRbt(g_world, g_currentPickedRbtNode);
    }

    if (drawArc)
    {
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)))
      {
        g_arcballScale = getScreenToEyeScale(z, g_frustFovY, g_windowHeight); // asst3, z를 현재 eye기준 object의 z로 계산해야됨
      }
      const float arcballEyeRadius = g_arcballScale * g_arcballScreenRadius;
      Matrix4 MVM = rigTFormToMatrix(invEyeRbt * g_arcballRbt) * Matrix4::makeScale(Cvec3(arcballEyeRadius));
      Matrix4 NMVM = normalMatrix(MVM);
      sendModelViewNormalMatrix(curSS, MVM, NMVM);
      safe_glUniform3f(curSS.h_uColor, g_arcballColor[0], g_arcballColor[1], g_arcballColor[2]);
      g_sphere->draw(curSS);
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
  }
  else
  {
    Picker picker(invEyeRbt, curSS);
    g_world->accept(picker);
    glFlush();
    g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
    g_nothingPicked = false;
    if (g_currentPickedRbtNode == g_groundNode)
      g_currentPickedRbtNode = shared_ptr<SgRbtNode>(); // set to NULL
    if (g_currentPickedRbtNode == NULL)
      g_nothingPicked = true;
  }
}

static void pick()
{
  // We need to set the clear color to black, for pick rendering.
  // so let's save the clear color
  GLdouble clearColor[4];
  glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

  glClearColor(0, 0, 0, 0);

  // using PICKING_SHADER as the shader
  glUseProgram(g_shaderStates[PICKING_SHADER]->program);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  drawStuff(*g_shaderStates[PICKING_SHADER], true);

  // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
  // to see result of the pick rendering pass
  // glutSwapBuffers();

  // Now set back the clear color
  glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

  checkGlErrors();
}

static void display()
{
  glUseProgram(g_shaderStates[g_activeShader]->program);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear framebuffer color&depth

  drawStuff(*g_shaderStates[g_activeShader], false);
  glutSwapBuffers(); // show the back buffer (where we rendered stuff)

  checkGlErrors();
}

static void reshape(const int w, const int h)
{
  g_windowWidth = w;
  g_windowHeight = h;
  glViewport(0, 0, w, h);
  cerr << "Size of window is now " << w << "x" << h << endl;
  g_arcballScreenRadius = min(g_windowHeight, g_windowWidth) * 0.25;
  updateFrustFovY();
  glutPostRedisplay();
}

static void makeAuxFrame()
{
  switch (viewpoint)
  {
  case 'l':
    auxilaryR = linFact(getPathAccumRbt(g_world, g_robot1Node));
    break;
  case 'r':
    auxilaryR = linFact(getPathAccumRbt(g_world, g_robot2Node));
    break;
  case 's':
    auxilaryR = linFact(getPathAccumRbt(g_world, g_skyNode));
    break;
  }

  if (g_currentPickedRbtNode == NULL)
  {
    if (wldSky == 's') // m에 따라 world-sky, sky-sky바꾸는거
    {
      auxilaryT = transFact(getPathAccumRbt(g_world, g_skyNode));
    }
    else
    {
      auxilaryT = RigTForm();
    }
  }
  else
  {
    auxilaryT = transFact(getPathAccumRbt(g_world, g_currentPickedRbtNode));
  }

  auxilaryFrame = auxilaryT * auxilaryR;
}

static double makeArcZ(int x, int y, Cvec2 arcCenter)
{
  int sq_x = (arcCenter[0] - x) * (arcCenter[0] - x);
  int sq_y = (arcCenter[1] - y) * (arcCenter[1] - y);
  int sq_r = g_arcballScreenRadius * g_arcballScreenRadius;
  // 마우스가 아크볼 밖을 클릭하면 z = 0으로 클립
  if (sq_x + sq_y > sq_r)
  {
    return 0;
  }
  return sqrt(sq_r - sq_x - sq_y);
}

static void motion(const int x, const int y)
{
  const double dx = x - g_mouseClickX; // 지금 마우스 위치 - 이전 프레임 위치 저장해둔거
  const double dy = g_windowHeight - y - 1 - g_mouseClickY;
  makeAuxFrame();

  RigTForm m;
  Cvec3 arcUnitVec_p, arcUnitVec_n; // previous, now
  if (g_mouseLClickButton && !g_mouseRClickButton)
  {
    if (drawArc)
    {
      Cvec2 arcScreenCenter = getScreenSpaceCoord((inv(eyeRbt) * auxilaryFrame).getTranslation(), makeProjectionMatrix(), 0, g_frustFovY, g_windowWidth, g_windowHeight);
      arcUnitVec_p = Cvec3(g_mouseClickX - arcScreenCenter[0], g_mouseClickY - arcScreenCenter[1], makeArcZ(g_mouseClickX, g_mouseClickY, arcScreenCenter)).normalize();
      arcUnitVec_n = Cvec3(x - arcScreenCenter[0], g_windowHeight - y - 1 - arcScreenCenter[1], makeArcZ(x, g_windowHeight - y - 1, arcScreenCenter)).normalize();
      m = RigTForm(Quat(0, arcUnitVec_n) * Quat(0, arcUnitVec_p * -1));
    }
    else
      m.setRotation(Quat::makeXRotation(-dy) * Quat::makeYRotation(dx));
  }
  else if (g_mouseRClickButton && !g_mouseLClickButton)
  {
    if (drawArc)
    {
      m.setTranslation(Cvec3(dx, dy, 0) * g_arcballScale);
    }
    else
    {
      m.setTranslation(Cvec3(dx, dy, 0) * 0.01);
    }
  }
  else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton))
  {
    if (drawArc)
    {
      m.setTranslation(Cvec3(0, 0, -dy) * g_arcballScale);
    }
    else
    {
      m.setTranslation(Cvec3(0, 0, -dy) * 0.01);
    }
  }

  if (g_mouseClickDown)
  {
    RigTForm afterMotion = RigTForm();
    if (g_currentPickedRbtNode == NULL || (viewpoint == 'l' && g_currentPickedRbtNode == g_robot1Node) || (viewpoint == 'r' && g_currentPickedRbtNode == g_robot2Node))
    {
      if (viewpoint == 's' && wldSky == 'w')
        m = inv(m);
      else
        m = transFact(m) * inv(linFact(m));
      switch (viewpoint)
      {
      case 's':
        g_skyNode->setRbt(auxilaryFrame * m * inv(auxilaryFrame) * g_skyNode->getRbt());
        break;
      case 'l':
        g_robot1Node->setRbt(auxilaryFrame * m * inv(auxilaryFrame) * g_robot1Node->getRbt());
        break;
      case 'r':
        g_robot2Node->setRbt(auxilaryFrame * m * inv(auxilaryFrame) * g_robot2Node->getRbt());
        break;
      }
    }
    else
    {
      // O = wA m A^-1 O, o = wO_-1 o = wO
      afterMotion = auxilaryFrame * m * inv(auxilaryFrame) * getPathAccumRbt(g_world, g_currentPickedRbtNode);
      g_currentPickedRbtNode->setRbt(inv(getPathAccumRbt(g_world, g_currentPickedRbtNode, 1)) * afterMotion);
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
  if (g_pickMode && g_mouseLClickButton)
  {
    pick();
    cout << g_currentPickedRbtNode << endl;
    cout << "Picking mode if off" << endl;
    g_pickMode = false;
  }
  glutPostRedisplay();
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
         << "v\t\tCycle view\n"
         << "m\t\tSwitching between world-sky and sky-sky frames for sky motion\n"
         << "p\t\tEnter picking mode to select object\n"
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
    // v눌리면 시점 전환: sky > left robot > right robot > sky
    switch (viewpoint)
    {
    case 's':
      viewpoint = 'l';
      break;
    case 'l':
      viewpoint = 'r';
      break;
    case 'r':
      viewpoint = 's';
      break;
    }
    break;
  case 'm':
    if (wldSky == 's')
    {
      cout << "Editing sky eye w.r.t. world-sky frame" << endl;
      wldSky = 'w';
    }
    else
    {
      cout << "Editing sky eye w.r.t. sky-sky frame" << endl;
      wldSky = 's';
    }
    break;
  case 'p':
    if (g_pickMode)
    {
      cout << "Picking mode is off" << endl;
      g_pickMode = false;
    }
    else
    {
      cout << "Picking mode is on" << endl;
      g_pickMode = true;
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
  glutCreateWindow("Assignment 4");                          // title the window

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
  initSphere();
}

static void constructRobot(shared_ptr<SgTransformNode> base, const Cvec3 &color)
{

  const float ARM_LEN = 0.7,
              ARM_THICK = 0.2,
              TORSO_LEN = 1.3,
              TORSO_THICK = 0.25,
              TORSO_WIDTH = 0.9,
              LEG_LEN = 0.8,
              LEG_THICK = 0.25,
              HEAD_RADIUS = 0.3;
  const int NUM_JOINTS = 10,
            NUM_SHAPES = 10;

  struct JointDesc
  {
    int parent;
    float x, y, z;
  };

  JointDesc jointDesc[NUM_JOINTS] = {
      {-1},                                     // torso
      {0, TORSO_WIDTH / 2, TORSO_LEN / 2, 0},   // upper right arm
      {1, ARM_LEN, 0, 0},                       // lower right arm
      {0, -TORSO_WIDTH / 2, TORSO_LEN / 2, 0},  // upper left arm
      {3, -ARM_LEN, 0, 0},                      // lower left arm
      {0, TORSO_WIDTH / 2, -TORSO_LEN / 2, 0},  // upper right leg
      {5, 0, -LEG_LEN, 0},                      // lower right leg
      {0, -TORSO_WIDTH / 2, -TORSO_LEN / 2, 0}, // upper left leg
      {7, 0, -LEG_LEN, 0},                      // lower left leg
      {0, 0, TORSO_LEN / 2, 0}                  // head
  };

  struct ShapeDesc
  {
    int parentJointId;
    float x, y, z, sx, sy, sz;
    shared_ptr<Geometry> geometry;
  };

  ShapeDesc shapeDesc[NUM_SHAPES] = {
      {0, 0, 0, 0, TORSO_WIDTH, TORSO_LEN, TORSO_THICK, g_cube},                      // torso
      {1, ARM_LEN / 2, 0, 0, ARM_LEN / 2, ARM_THICK / 2, ARM_THICK / 2, g_sphere},    // upper right arm
      {2, ARM_LEN / 2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube},                  // lower right arm
      {3, -ARM_LEN / 2, 0, 0, ARM_LEN / 2, ARM_THICK / 2, ARM_THICK / 2, g_sphere},   // upper left arm
      {4, -ARM_LEN / 2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube},                 // lower left arm
      {5, 0, -LEG_LEN / 2, 0, LEG_THICK / 2, LEG_LEN / 2, LEG_THICK / 2, g_sphere},   // upper right leg
      {6, 0, -LEG_LEN / 2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube},                 // lower right leg
      {7, 0, -LEG_LEN / 2, 0, LEG_THICK / 2, LEG_LEN / 2, LEG_THICK / 2, g_sphere},   // upper left leg
      {8, 0, -LEG_LEN / 2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube},                 // lower left leg
      {9, 0, HEAD_RADIUS * 1.2f, 0, HEAD_RADIUS, HEAD_RADIUS, HEAD_RADIUS, g_sphere}, // head
  };

  shared_ptr<SgTransformNode> jointNodes[NUM_JOINTS];

  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    if (jointDesc[i].parent == -1)
      jointNodes[i] = base;
    else
    {
      jointNodes[i].reset(new SgRbtNode(RigTForm(Cvec3(jointDesc[i].x, jointDesc[i].y, jointDesc[i].z))));
      jointNodes[jointDesc[i].parent]->addChild(jointNodes[i]);
    }
  }
  for (int i = 0; i < NUM_SHAPES; ++i)
  {
    shared_ptr<MyShapeNode> shape(
        new MyShapeNode(shapeDesc[i].geometry,
                        color,
                        Cvec3(shapeDesc[i].x, shapeDesc[i].y, shapeDesc[i].z),
                        Cvec3(0, 0, 0),
                        Cvec3(shapeDesc[i].sx, shapeDesc[i].sy, shapeDesc[i].sz)));
    jointNodes[shapeDesc[i].parentJointId]->addChild(shape);
  }
}

static void initScene()
{
  g_world.reset(new SgRootNode());

  g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 4.0))));

  g_groundNode.reset(new SgRbtNode());
  g_groundNode->addChild(shared_ptr<MyShapeNode>(
      new MyShapeNode(g_ground, Cvec3(0.1, 0.95, 0.1))));

  g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
  g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

  constructRobot(g_robot1Node, Cvec3(1, 0, 0)); // a Red robot
  constructRobot(g_robot2Node, Cvec3(0, 0, 1)); // a Blue robot

  g_world->addChild(g_skyNode);
  g_world->addChild(g_groundNode);
  g_world->addChild(g_robot1Node);
  g_world->addChild(g_robot2Node);
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
    initScene();

    glutMainLoop();
    return 0;
  }
  catch (const runtime_error &e)
  {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}
