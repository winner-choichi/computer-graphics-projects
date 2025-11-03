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
#include "geometry.h"
#include "mesh.h"

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

// Animation, Frame & Script Support
#include "keyframes.h"
#include "interpolation.h"

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
bool g_is_mesh_smooth = false;
bool g_shading_toggle_pending = false;
bool g_subdivision_pending = false;

static RigTForm auxilaryFrame, auxilaryT, auxilaryR, eyeRbt;

static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_light1Node, g_light2Node, g_groundNode, g_robot1Node, g_robot2Node;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode; // used later when you do picking
static shared_ptr<SgRbtNode> g_animation_cube;

// --------- Materials
static shared_ptr<Material> g_redDiffuseMat,
    g_blueDiffuseMat,
    g_bumpFloorMat,
    g_arcballMat,
    g_pickingMat,
    g_lightMat,
    g_specMat;

shared_ptr<Material> g_overridingMaterial;

static shared_ptr<Script> g_script;
static const std::string SCRIPT_FILE = "animation.txt";

static bool g_animating = false;
static int g_msBetweenKeyFrames = 2000;   // 2 seconds between keyframes
static int g_animateFramesPerSecond = 60; // frames to render per second during animation playback

float g_mesh_animation_speed = 0.01;
int g_start_time_ms = -1;

// --------- Geometry
typedef SgGeometryShapeNode MyShapeNode;

static shared_ptr<Geometry> g_ground, g_cube, g_sphere; // Vertex buffer and index buffer associated with the ground and cube geometry

static shared_ptr<Mesh> g_mesh = make_shared<Mesh>();
static shared_ptr<Mesh> g_subdivided_mesh = make_shared<Mesh>();
static std::shared_ptr<SimpleGeometryPN> g_mesh_geom_pn;
static int g_mesh_resolution_lv = 0;

// ============================================================================
// Interface
// ============================================================================

static RigTForm g_arcballRbt;
static Cvec3f g_arcballColor = Cvec3f(0, 1, 0);

///////////////// END OF G L O B A L S //////////////////////////////////////////////////

static void initGround()
{
  int ibLen, vbLen;
  getPlaneVbIbLen(vbLen, ibLen);

  // Temporary storage for cube Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makePlane(g_groundSize * 2, vtx.begin(), idx.begin());
  g_ground.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initCubes()
{
  int ibLen, vbLen;
  getCubeVbIbLen(vbLen, ibLen);

  // Temporary storage for cube Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeCube(1, vtx.begin(), idx.begin());
  g_cube.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initSphere()
{
  int ibLen, vbLen;
  getSphereVbIbLen(20, 10, vbLen, ibLen);

  // Temporary storage for sphere Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);
  makeSphere(1, 20, 10, vtx.begin(), idx.begin());
  g_sphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

static void initMeshCube()
{
  g_mesh_geom_pn = std::make_shared<SimpleGeometryPN>();
  // Load the mesh from file
  g_mesh->load("data/cube.mesh");

  vector<VertexPN> vtx;

  for (int i = 0; i < g_mesh->getNumFaces(); ++i)
  {
    Mesh::Face f = g_mesh->getFace(i);
    std::vector<Cvec3> pos;
    for (int j = 0; j < f.getNumVertices(); ++j)
      pos.push_back(f.getVertex(j).getPosition());

    // from quad to triangle1
    vtx.push_back(VertexPN(pos[0], f.getNormal())); // face normal for flat shading
    vtx.push_back(VertexPN(pos[1], f.getNormal())); // face normal for flat shading
    vtx.push_back(VertexPN(pos[2], f.getNormal())); // face normal for flat shading

    // from quad to triangle2
    vtx.push_back(VertexPN(pos[2], f.getNormal())); // face normal for flat shading
    vtx.push_back(VertexPN(pos[3], f.getNormal())); // face normal for flat shading
    vtx.push_back(VertexPN(pos[0], f.getNormal())); // face normal for flat shading
  }

  g_mesh_geom_pn->upload(&vtx[0], vtx.size());
  // cout << "size: " << vtx.size() << endl;
}

shared_ptr<Mesh> catmull_clark_subdivision_mesh(shared_ptr<Mesh> m)
{
  shared_ptr<Mesh> result_mesh = make_shared<Mesh>(*m);

  // FaceVertex iteration
  for (int f = 0; f < result_mesh->getNumFaces(); ++f)
  {
    Mesh::Face f_temp = result_mesh->getFace(f);
    Cvec3 pos(0);
    for (int v = 0; v < f_temp.getNumVertices(); ++v)
    {
      Mesh::Vertex v_temp = f_temp.getVertex(v);
      pos += v_temp.getPosition();
    }
    pos /= f_temp.getNumVertices();
    result_mesh->setNewFaceVertex(f_temp, pos);
  }

  // EdgeVertex iteration
  for (int e = 0; e < result_mesh->getNumEdges(); ++e)
  {
    Mesh::Edge e_temp = result_mesh->getEdge(e);
    Cvec3 pos(0);
    pos += e_temp.getVertex(0).getPosition();
    pos += e_temp.getVertex(1).getPosition();
    pos += result_mesh->getNewFaceVertex(e_temp.getFace(0));
    pos += result_mesh->getNewFaceVertex(e_temp.getFace(1));
    pos /= 4.0;
    result_mesh->setNewEdgeVertex(e_temp, pos);
  }

  // VertexVertex iteration
  for (int v = 0; v < result_mesh->getNumVertices(); ++v)
  {
    Mesh::Vertex v_temp = result_mesh->getVertex(v);
    Cvec3 F(0), V(0);
    int n = 0;

    Mesh::VertexIterator iter(v_temp.getIterator()), it0(iter);
    do
    {
      F += result_mesh->getNewFaceVertex(iter.getFace());
      V += iter.getVertex().getPosition();
      ++n;
    } while (++iter != it0);
    Cvec3 pos = v_temp.getPosition() * (((double)n - 2.0) / (double)n) + V * (1.0 / (double)pow(n, 2)) + F * (1.0 / (double)pow(n, 2));
    result_mesh->setNewVertexVertex(v_temp, pos);
  }

  // Apply cached new vertices to subdivision
  result_mesh->subdivide();

  // vertex normal 초기화: flat shading용 normal로 대충 박기
  for (int i = 0; i < result_mesh->getNumFaces(); ++i)
  {
    Mesh::Face f = result_mesh->getFace(i);
    for (int j = 0; j < f.getNumVertices(); ++j)
    {
      f.getVertex(j).setNormal(f.getNormal());
    }
  }

  return result_mesh;
}

static void toggle_mesh_shading(shared_ptr<Mesh> mesh, bool smooth)
{
  shared_ptr<Mesh> temp_mesh = mesh;
  bool subdivided_mesh_selected = false;
  if (g_mesh_resolution_lv != 0)
  {
    subdivided_mesh_selected = true;
  }
  // cout << "# of divided mesh: " << temp_mesh->getNumVertices() << endl;

  vector<VertexPN> vtx;
  if (smooth)
  {
    // first reset all vertices normals to zero
    for (int i = 0; i < temp_mesh->getNumFaces(); ++i)
    {
      Mesh::Face f = temp_mesh->getFace(i);
      for (int j = 0; j < f.getNumVertices(); ++j)
      {
        f.getVertex(j).setNormal(Cvec3(0, 0, 0));
      }
    }

    // and then accumulate
    for (int i = 0; i < temp_mesh->getNumFaces(); ++i)
    {
      Mesh::Face f = temp_mesh->getFace(i);
      for (int j = 0; j < f.getNumVertices(); ++j)
      {
        f.getVertex(j).setNormal(f.getVertex(j).getNormal() + f.getNormal());
      }
    }

    // lastely divide them with valance of a vertex
    for (int i = 0; i < temp_mesh->getNumVertices(); ++i)
    {
      const Mesh::Vertex v = temp_mesh->getVertex(i);

      int count = 0;
      Mesh::VertexIterator it(v.getIterator()), it0(it);
      do
      {
        ++count;
      } while (++it != it0); // go around once the 1ring

      v.setNormal(v.getNormal() / count);
    }

    for (int i = 0; i < temp_mesh->getNumFaces(); ++i)
    {
      Mesh::Face f = temp_mesh->getFace(i);
      std::vector<Cvec3> pos;
      for (int j = 0; j < f.getNumVertices(); ++j)
        pos.push_back(f.getVertex(j).getPosition());

      // from quad to triangle1
      vtx.push_back(VertexPN(pos[0], f.getVertex(0).getNormal()));
      vtx.push_back(VertexPN(pos[1], f.getVertex(1).getNormal()));
      vtx.push_back(VertexPN(pos[2], f.getVertex(2).getNormal()));

      // from quad to triangle2
      vtx.push_back(VertexPN(pos[2], f.getVertex(2).getNormal()));
      vtx.push_back(VertexPN(pos[3], f.getVertex(3).getNormal()));
      vtx.push_back(VertexPN(pos[0], f.getVertex(0).getNormal()));
    }
  }
  else
  {
    for (int i = 0; i < temp_mesh->getNumFaces(); ++i)
    {
      Mesh::Face f = temp_mesh->getFace(i);
      std::vector<Cvec3> pos;
      for (int j = 0; j < f.getNumVertices(); ++j)
        pos.push_back(f.getVertex(j).getPosition());

      // from quad to triangle1
      vtx.push_back(VertexPN(pos[0], f.getNormal())); // face normal for flat shading
      vtx.push_back(VertexPN(pos[1], f.getNormal())); // face normal for flat shading
      vtx.push_back(VertexPN(pos[2], f.getNormal())); // face normal for flat shading

      // from quad to triangle2
      vtx.push_back(VertexPN(pos[2], f.getNormal())); // face normal for flat shading
      vtx.push_back(VertexPN(pos[3], f.getNormal())); // face normal for flat shading
      vtx.push_back(VertexPN(pos[0], f.getNormal())); // face normal for flat shading
    }
  }

  g_mesh_geom_pn->upload(&vtx[0], vtx.size());
}

shared_ptr<Mesh> subdivide_nth_catmullclark(shared_ptr<Mesh> mesh, int n)
{
  shared_ptr<Mesh> temp = make_shared<Mesh>(*mesh);
  for (int i = 0; i < n; ++i)
  {
    temp = catmull_clark_subdivision_mesh(temp);
  }

  g_subdivided_mesh = temp;

  return g_subdivided_mesh;
}

shared_ptr<Mesh> meshPointsRescale(float time)
{
  auto hash = [](int i) -> float
  {
    float x = sin(i * 123.4f) * 20210.663f;
    return x - floor(x);
  };

  shared_ptr<Mesh> temp = make_shared<Mesh>(*g_mesh);
  for (int i = 0; i < temp->getNumVertices(); ++i)
  {
    float phase = hash(i) * CS175_PI * 2.0f;
    float scale_factor = 1.0f + sin(phase + 500.0 * g_mesh_animation_speed * time);
    temp->getVertex(i).setPosition(g_mesh->getVertex(i).getPosition() * scale_factor);
  }

  return temp;
}

void animateMeshTimerCallback(int)
{
  // if (g_shading_toggle_pending)
  // {
  //   toggle_mesh_shading(g_is_mesh_smooth);
  //   g_shading_toggle_pending = false;
  // }

  if (g_start_time_ms == -1)
    g_start_time_ms = glutGet(GLUT_ELAPSED_TIME);

  float elapsed_sec = (glutGet(GLUT_ELAPSED_TIME) - g_start_time_ms) / 1000.0f;
  shared_ptr<Mesh> temp = make_shared<Mesh>();
  temp = meshPointsRescale(elapsed_sec);

  if (g_subdivision_pending)
  {
    temp = subdivide_nth_catmullclark(temp, g_mesh_resolution_lv);
  }
  toggle_mesh_shading(temp, g_is_mesh_smooth);

  glutTimerFunc(1000 / 60, animateMeshTimerCallback, 0);
  glutPostRedisplay();
}

// takes a projection matrix and send to the the shaders
inline void sendProjectionMatrix(Uniforms &uniforms, const Matrix4 &projMatrix)
{
  uniforms.put("uProjMatrix", projMatrix);
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

static void drawStuff(bool picking)
{
  Uniforms uniforms;
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(uniforms, projmat);

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
  const Cvec3 eyeLight1 = (invEyeRbt * getPathAccumRbt(g_world, g_light1Node)).getTranslation();
  const Cvec3 eyeLight2 = (invEyeRbt * getPathAccumRbt(g_world, g_light2Node)).getTranslation();

  // // send the eye space coordinates of lights to uniforms
  uniforms.put("uLight", eyeLight1);
  uniforms.put("uLight2", eyeLight2);

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
    Drawer drawer(invEyeRbt, uniforms);
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
      if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)))
      {
        g_arcballScale = getScreenToEyeScale(z, g_frustFovY, g_windowHeight); // asst3, z를 현재 eye기준 object의 z로 계산해야됨
      }
      const float arcballEyeRadius = g_arcballScale * g_arcballScreenRadius;
      Matrix4 MVM = rigTFormToMatrix(invEyeRbt * g_arcballRbt) * Matrix4::makeScale(Cvec3(arcballEyeRadius));
      Matrix4 NMVM = normalMatrix(MVM);
      sendModelViewNormalMatrix(uniforms, MVM, NMVM);
      g_arcballMat->draw(*g_sphere, uniforms);
    }
  }
  else
  {
    Picker picker(invEyeRbt, uniforms);
    g_overridingMaterial = g_pickingMat; // set overiding material to our picking material
    g_world->accept(picker);
    g_overridingMaterial.reset(); // unset the overriding material
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

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  drawStuff(true); // no more curSS

  // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
  // to see result of the pick rendering pass
  // glutSwapBuffers();

  // Now set back the clear color
  glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

  checkGlErrors();
}

static void display()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  drawStuff(false); // no more curSS

  glutSwapBuffers();

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

static bool interpolateAndDisplay(float t)
{
  int i = floor(t);
  float a = t - i;

  if (g_script->get_script_size() < 2 || t > g_script->get_script_size() - 2) // n-1까지만
    return true;

  Frame f0 = g_script->get_frame_at(i);
  Frame f1 = g_script->get_frame_at(i + 1);
  Frame fa = get_frame_interpolation(f0, f1, a);
  fa.paste_to_scene(g_world);
  glutPostRedisplay();
  return false;
}

static void animateTimerCallback(int ms)
{
  float t = (float)ms / (float)g_msBetweenKeyFrames; // 지금 시간 / 프레임당 시간 -> t
  bool endReached = interpolateAndDisplay(t);

  if (!endReached)
  {
    // 1초에 몇 번씩 호출할지 / 어떤 함수 호출할지 / 호출하는 시간 ms
    glutTimerFunc(1000 / g_animateFramesPerSecond, animateTimerCallback, ms + 1000 / g_animateFramesPerSecond);
  }
  else
  {
    g_script->set_currframe_to_n_display(g_script->get_script_size() - 2);
    g_animating = false;
  }
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

  // Script hot keys
  case ' ':
    if (g_animating)
    {
      cout << "Cannot operate when playing animation" << endl;
      break;
    }
    g_script->paste_curr_frame();
    break;
  case 'u':
    if (g_animating)
    {
      break;
    }
    g_script->update_frame();
    break;
  case '>':
    if (g_animating)
    {
      cout << "Cannot operate when playing animation" << endl;
      break;
    }
    g_script->move_next_frame();
    break;
  case '<':
    if (g_animating)
    {
      cout << "Cannot operate when playing animation" << endl;
      break;
    }
    g_script->move_prev_frame();
    break;
  case 'd':
    if (g_animating)
    {
      break;
    }
    g_script->delete_frame();
    break;
  case 'n':
    if (g_animating)
    {
      break;
    }
    g_script->add_frame();
    break;
  case 'i':
    if (g_animating)
    {
      break;
    }
    g_script->set_script(SCRIPT_FILE);
    break;
  case 'w':
    g_script->get_script(SCRIPT_FILE);
    break;

  // Animation hot keys
  case 'y':
    if (g_animating)
    {
      cout << "Already animating" << endl;
      break;
    }
    if (g_script->get_script_size() >= 4)
    {
      interpolateAndDisplay(0.0f);
      animateTimerCallback(0);
      g_animating = true;
    }
    else
    {
      cout << "Need at least 4 keyframes" << endl;
    }
    break;
  case '+':
    g_msBetweenKeyFrames = max(100, g_msBetweenKeyFrames - 100);
    cout << "Speed up: " << g_msBetweenKeyFrames << "ms per keyframe" << endl;
    break;
  case '-':
    g_msBetweenKeyFrames = min(3000, g_msBetweenKeyFrames + 100);
    cout << "Slow down: " << g_msBetweenKeyFrames << "ms per keyframe" << endl;
    break;
  // Mesh animation hot keys
  case 'f':
    g_is_mesh_smooth = !g_is_mesh_smooth;
    g_shading_toggle_pending = true;
    cout << "toggle smooth shading: " << (g_is_mesh_smooth ? "true" : "false") << endl;
    break;
  case '0':
    if (g_mesh_resolution_lv < 7)
    {
      ++g_mesh_resolution_lv;
      g_subdivision_pending = true;
    }
    cout << "Subdivision levels = " << g_mesh_resolution_lv << endl;
    break;
  case '9':
    if (g_mesh_resolution_lv > 0)
    {
      --g_mesh_resolution_lv;
      g_subdivision_pending = true;
      if (g_mesh_resolution_lv == 0)
        g_subdivision_pending = false;
    }
    else
    {
      g_subdivision_pending = false;
    }
    cout << "Subdivision levels = " << g_mesh_resolution_lv << endl;
    break;
  case '8':
  {
    float old_speed = g_mesh_animation_speed;
    float new_speed = 2.0 * old_speed;
    int curr_time = glutGet(GLUT_ELAPSED_TIME);
    float elapsed_sec = (curr_time - g_start_time_ms) / 1000.0f;

    // 속도 바꾼 후에도 같은 phase 유지하도록 기준 시간 조정
    g_start_time_ms = curr_time - static_cast<int>((elapsed_sec * old_speed / new_speed) * 1000.0f);
    g_mesh_animation_speed = new_speed;
    cout << "Bubbling speed: " << g_mesh_animation_speed << endl;
    break;
  }
  case '7':
  {
    float old_speed = g_mesh_animation_speed;
    float new_speed = old_speed / 2.0;
    int curr_time = glutGet(GLUT_ELAPSED_TIME);
    float elapsed_sec = (curr_time - g_start_time_ms) / 1000.0f;

    // 속도 줄인 후에도 같은 phase 유지
    g_start_time_ms = curr_time - static_cast<int>((elapsed_sec * old_speed / new_speed) * 1000.0f);
    g_mesh_animation_speed = new_speed;
    cout << "Bubbling speed: " << g_mesh_animation_speed << endl;
    break;
  }
    glutPostRedisplay();
  }
}

static void initGlutState(int argc, char *argv[])
{
  glutInit(&argc, argv);                                     // initialize Glut based on cmd-line args
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH); //  RGBA pixel channels and double buffering
  glutInitWindowSize(g_windowWidth, g_windowHeight);         // create a window
  glutCreateWindow("Assignment 6");                          // title the window

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

static void initMaterials()
{
  // Create some prototype materials
  Material diffuse("./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader");
  Material solid("./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader");
  Material normal("./shaders/normal-gl3.vshader", "./shaders/normal-gl3.fshader");
  Material specular("./shaders/basic-gl3.vshader", "./shaders/specular-gl3.fshader");

  // copy diffuse prototype and set red color
  g_redDiffuseMat.reset(new Material(diffuse));
  g_redDiffuseMat->getUniforms().put("uColor", Cvec3f(1, 0, 0));

  // copy diffuse prototype and set blue color
  g_blueDiffuseMat.reset(new Material(diffuse));
  g_blueDiffuseMat->getUniforms().put("uColor", Cvec3f(0, 0, 1));

  // normal mapping material
  g_bumpFloorMat.reset(new Material(normal));
  g_bumpFloorMat->getUniforms().put("uTexColor", shared_ptr<ImageTexture>(new ImageTexture("Fieldstone.ppm", true)));
  g_bumpFloorMat->getUniforms().put("uTexNormal", shared_ptr<ImageTexture>(new ImageTexture("FieldstoneNormal.ppm", false)));

  // copy solid prototype, and set to wireframed rendering
  g_arcballMat.reset(new Material(solid));
  g_arcballMat->getUniforms().put("uColor", Cvec3f(0.27f, 0.82f, 0.35f));
  g_arcballMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_LINE);

  // copy solid prototype, and set to color white
  g_lightMat.reset(new Material(solid));
  g_lightMat->getUniforms().put("uColor", Cvec3f(1, 1, 1));

  // pick shader
  g_pickingMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"));

  // Mesh specualr material
  g_specMat.reset(new Material(specular));
  g_specMat->getUniforms().put("uColor", Cvec3f(0.5, 0.5, 0));
}

static void initGeometry()
{
  initGround();
  initCubes();
  initSphere();
  initMeshCube();
}

static void constructRobot(shared_ptr<SgTransformNode> base, shared_ptr<Material> material)
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
                        material,
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

  g_light1Node.reset(new SgRbtNode(RigTForm(Cvec3(4.0, 3.0, 5.0))));
  g_light1Node->addChild(shared_ptr<MyShapeNode>(new MyShapeNode(g_sphere, g_lightMat, Cvec3(0, 0, 0), Cvec3(0, 0, 0), Cvec3(0.5, 0.5, 0.5))));
  g_light2Node.reset(new SgRbtNode(RigTForm(Cvec3(-4, 1.0, -5.0))));
  g_light2Node->addChild(shared_ptr<MyShapeNode>(new MyShapeNode(g_sphere, g_lightMat, Cvec3(0, 0, 0), Cvec3(0, 0, 0), Cvec3(0.5, 0.5, 0.5))));

  g_groundNode.reset(new SgRbtNode());
  g_groundNode->addChild(shared_ptr<MyShapeNode>(
      new MyShapeNode(g_ground, g_bumpFloorMat, Cvec3(0, g_groundY, 0))));

  g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
  g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

  constructRobot(g_robot1Node, g_redDiffuseMat);  // a Red robot
  constructRobot(g_robot2Node, g_blueDiffuseMat); // a Blue robot

  g_animation_cube.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.5, 0.0))));
  g_animation_cube->addChild(shared_ptr<MyShapeNode>(new MyShapeNode(g_mesh_geom_pn, g_specMat, Cvec3(0, 0, 0), Cvec3(0, 0, 0))));

  g_world->addChild(g_skyNode);
  g_world->addChild(g_light1Node);
  g_world->addChild(g_light2Node);
  g_world->addChild(g_groundNode);
  g_world->addChild(g_robot1Node);
  g_world->addChild(g_robot2Node);
  g_world->addChild(g_animation_cube);

  animateMeshTimerCallback(0);
}

static void initScript()
{
  g_script = make_shared<Script>(g_world);
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
    initMaterials();
    initGeometry();
    initScene();
    initScript();

    glutMainLoop();
    return 0;
  }
  catch (const runtime_error &e)
  {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}
