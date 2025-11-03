# Computer Graphics - Course Projects

This repository contains OpenGL-based computer graphics projects completed during a semester-long graphics course. The assignments progressively build upon each other, covering fundamental graphics concepts from basic rendering to advanced mesh subdivision.

## Course Information
- **Course**: CS380 Introduction to Computer Graphics
- **Institution**: KAIST
- **Semester**: 2025 Spring
- **Framework**: OpenGL with GLUT/GLEW
- **Language**: C++
- **Shading**: GLSL (OpenGL Shading Language)

---

## Project Overview

### HW1: Introduction to OpenGL and Shaders
**Files**: `hw1/asst1-2025/asst1.cpp`

**Implemented Features**:
- Basic OpenGL rendering setup with GLUT
- Vertex and fragment shader integration
- Texture mapping (spinning cat texture)
- Time-based animations using `glutGet(GLUT_ELAPSED_TIME)`
- Window resizing and aspect ratio handling
- Mouse interaction for object scaling
- Screenshot saving functionality (PPM format)

**Key Concepts**:
- Vertex Buffer Objects (VBOs)
- Shader uniforms (time, window size, vertex scale)
- Basic 2D geometry rendering
- Animation loop with `glutIdleFunc`

---

### HW2: 3D Transformations and Camera Control
**Files**: `hw2/asst2/asst2.cpp`

**Implemented Features**:
- 3D scene with ground plane and two cubes
- Multiple viewpoint switching (Sky, Left Cube, Right Cube) - press `v`
- Multiple object control switching - press `o`
- Camera manipulation:
  - Left mouse drag: Rotation
  - Right mouse drag: Translation (XY plane)
  - Middle mouse drag: Translation (Z-axis/depth)
- Diffuse and solid shading modes (toggle with `f`)
- World-sky vs Sky-sky frame switching (press `m`)
- Auxiliary frame system for proper object transformations
- Projection matrix with field-of-view calculations

**Key Concepts**:
- Matrix transformations (rotation, translation)
- Eye coordinate system vs World coordinate system
- Model-View-Projection (MVP) pipeline
- Normal matrix calculations for lighting
- Multiple object hierarchy

---

### HW3: Quaternions and Arcball Interface
**Files**: `hw3/asst3/asst3.cpp`

**Implemented Features**:
- RigTForm (Rigid Body Transformation) using quaternions
- Virtual arcball sphere for intuitive 3D rotation
- Screen-space to 3D sphere projection
- Arcball visual feedback (wireframe sphere)
- Dynamic arcball scaling based on object depth
- Sphere geometry generation
- Improved rotation controls with arcball metaphor

**Key Concepts**:
- Quaternion mathematics for rotation
- Arcball interface algorithm
- Screen-to-eye coordinate transformation
- Gimbal-lock-free rotations
- Linear and translational factorization of RigTForm

---

### HW4: Scene Graph and Hierarchical Modeling
**Files**: `hw4/asst4/asst4.cpp`

**Implemented Features**:
- Scene graph architecture (tree structure)
- Hierarchical robot models with articulated joints
  - Torso, arms (upper/lower), legs (upper/lower), head
  - 10 joints per robot with parent-child relationships
- Object picking system using GPU color-based picking
- Visitor pattern for scene traversal (Drawer, Picker)
- Two robots with different colors (red and blue)
- Node-based transformation accumulation
- Interactive object selection (press `p` for picking mode)

**Key Concepts**:
- Scene graph data structure
- Node transformation hierarchy
- Path accumulation for world transformations
- GPU-based object picking with unique color IDs
- Visitor design pattern
- Articulated figure modeling

---

### HW5: Keyframe Animation and Interpolation
**Files**: `hw5/asst5/asst5.cpp`

**Implemented Features**:
- Keyframe-based animation system
- Script management for animation sequences
- Frame interpolation using quaternion slerp
- Animation playback controls:
  - Play/pause animation
  - Frame advance/reverse
  - Keyframe creation and deletion
- File I/O for saving/loading animation scripts (`animation.txt`)
- Time-based interpolation (2 seconds between keyframes)
- 60 FPS animation playback

**Key Concepts**:
- Keyframe animation
- Quaternion spherical linear interpolation (slerp)
- Animation timing and frame rate control
- Pose interpolation
- Animation script serialization

---

### HW6: Material System and Texture Mapping
**Files**: `hw6/asst6/asst6.cpp`

**Implemented Features**:
- Advanced material system with multiple materials:
  - Diffuse materials (red, blue)
  - Bump-mapped floor material
  - Arcball material
  - Picking material
  - Light source material
- Enhanced geometry with tangent-space basis (PNTBX vertices)
- Texture coordinate generation
- Normal mapping/bump mapping support
- Multiple light sources in the scene
- Material-based rendering with uniforms system
- Improved ground plane with texture support

**Key Concepts**:
- Material abstraction
- Texture mapping with tangent-space
- Normal mapping for surface detail
- Per-vertex tangent and bitangent vectors
- Uniforms management system
- Multi-light rendering

---

### HW7: Basic Animation (Continued)
**Files**: `hw7/asst7/asst7.cpp`

**Implemented Features**:
- Refined animation system from HW5
- Smooth keyframe transitions
- Improved animation controls
- Scene graph integration with animation
- Per-node animation support

**Key Concepts**:
- Animation system refinement
- Scene graph animation
- Temporal coherence

---

### HW8: Mesh Subdivision and Smooth Surfaces
**Files**: `hw8/asst8/asst8.cpp`

**Implemented Features**:
- Mesh data structure with half-edge topology
- Catmull-Clark subdivision algorithm
  - Face vertex computation (average of face vertices)
  - Edge vertex computation (average of adjacent vertices and face centers)
  - Vertex vertex computation (weighted average formula)
- Multiple subdivision levels (0-6 levels)
- Smooth vs flat shading toggle:
  - Flat shading: Face normals
  - Smooth shading: Vertex normals (averaged from adjacent faces)
- Mesh loading from file (`data/cube.mesh`)
- Dynamic mesh geometry updates
- Real-time mesh manipulation

**Key Concepts**:
- Catmull-Clark subdivision surfaces
- Half-edge mesh representation
- Vertex normal computation
- Face/edge/vertex topology
- Smooth surface generation
- Mesh iteration patterns

**Subdivision Formula**:
```
V' = V * (n-2)/n + (1/n²) * Σ(adjacent vertices) + (1/n²) * Σ(adjacent face centers)
```

---

## Build Instructions

Each assignment has its own directory with a similar structure:
```
hwX/asstX/
├── asst.cpp          # Main source file
├── shaders/          # GLSL shader files
├── Makefile          # Build configuration
└── ...               # Supporting files
```

### Prerequisites
- OpenGL 2.x or 3.x compatible graphics card
- GLUT/FreeGLUT
- GLEW (OpenGL Extension Wrangler)
- C++11 compatible compiler

### Compilation
```bash
cd hwX/asstX
make
./asst
```

---

## Controls Summary

### Common Controls (Most Assignments)
- `h` - Display help menu
- `s` - Save screenshot (out.ppm)
- `q` / `ESC` - Quit application
- `v` - Cycle viewpoint (Sky → Left Object → Right Object)
- `o` - Cycle controlled object
- `f` - Toggle flat/smooth shading
- `m` - Toggle world-sky / sky-sky frame (for sky manipulation)
- `p` - Enter picking mode (HW4+)

### Mouse Controls
- **Left drag**: Rotate object/view
- **Right drag**: Translate in XY plane
- **Middle drag** or **Left+Right drag**: Translate in Z-axis

### Animation Controls (HW5, HW7)
- `Space` - Play/pause animation
- `y` - Start animating
- `u` - Copy current frame to scene
- `>` - Advance to next keyframe
- `<` - Go to previous keyframe
- `d` - Delete current keyframe
- `n` - Create new keyframe
- `i` - Load animation script
- `w` - Save animation script

### Mesh Controls (HW8)
- `0-6` - Set subdivision level
- `f` - Toggle smooth/flat shading
- `+/-` - Adjust animation speed (if applicable)

---

## Technical Highlights

### Graphics Pipeline
1. **Vertex Processing**: Transform vertices through Model-View-Projection matrices
2. **Rasterization**: Convert primitives to fragments
3. **Fragment Shading**: Apply lighting, textures, and materials
4. **Output**: Display to framebuffer

### Lighting Model
- Multiple light sources (two point lights)
- Diffuse shading (Lambertian reflectance)
- Normal transformation for proper lighting in eye space

### Transformation Hierarchy
```
World Space
  ├── Sky/Camera
  ├── Ground
  ├── Robot 1
  │   ├── Torso
  │   ├── Arms (left/right, upper/lower)
  │   ├── Legs (left/right, upper/lower)
  │   └── Head
  └── Robot 2 (same structure)
```

---

## Mathematical Foundations

### RigTForm (Rigid Body Transform)
Combines rotation (quaternion) and translation (vector):
```
RigTForm = Translation ∘ Rotation
```

### Quaternion Rotation
- Avoids gimbal lock
- Smooth interpolation via slerp
- Efficient composition

### Catmull-Clark Subdivision
Generates smooth surfaces from coarse meshes:
- **Face points**: Average of face vertices
- **Edge points**: Average of edge endpoints and adjacent face points
- **Vertex points**: Weighted average incorporating valence

---

## File Formats

### PPM (Portable Pixmap)
Screenshots saved in ASCII PPM format (P3):
```
P3
width height
255
R G B R G B ...
```

### Animation Script
Custom text format storing:
- Number of frames
- RigTForm data (translation + quaternion) for each node per frame

### Mesh Format
Custom format with vertices, faces, and connectivity information

---

## Learning Outcomes

Through these assignments, I gained practical experience in:
- OpenGL rendering pipeline and state management
- Shader programming (GLSL)
- 3D mathematics (matrices, quaternions, projections)
- Scene graph architecture and design patterns
- Geometric algorithms (subdivision surfaces)
- Interactive 3D applications
- Computer graphics theory and practice

---

## License

Educational project - Course assignment work
