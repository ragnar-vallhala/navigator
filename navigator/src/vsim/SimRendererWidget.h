/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

// The procedural grass/flora renderer is gated behind VAYU_SIM_GRASS (a CMake
// option, OFF by default). When off, GpuGrass.cpp + procgen/Flora.cpp are not
// compiled and every grass/flora entry point below is stubbed to a no-op, so
// the SITL view renders terrain without grass. The code is retained in-tree for
// later re-enable (cmake -DNAVIGATOR_SIM_GRASS=ON).
#ifdef VAYU_SIM_GRASS
#include "GpuGrass.h"
#endif
#include "DroneModel.h"
#include "SimWorker.h"
#include "TrainingCourse.h"

#include <QElapsedTimer>
#include <QMatrix4x4>
#include <QString>
#include <QOpenGLBuffer>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QImage>
#include <QOpenGLWindow>
#include <QPointer>
#include <QPoint>

#include <array>
#include <map>
#include <memory>
#include <vector>

namespace vsim {

// OpenGL 3.3 core profile renderer for the in-app sim. Draws the
// drone in NED world coordinates directly (camera "up" vector is
// (0,0,-1)), so the world axes you see line up with what the IMU /
// firmware actually sees.
//
// This is a QOpenGLWindow, not a QOpenGLWidget: host it with
// QWidget::createWindowContainer(renderer, parent). A QOpenGLWidget renders
// into the top-level's backing store, which breaks presentation of the main
// window when its panel floats under the docking system; a window container
// gives the view its own native surface and context. Consequences for
// callers: overlay widgets cannot be stacked over it (a native window is
// always on top), so the HUD is painted by the renderer itself (setHudWidget)
// and picture-in-picture panes sit beside it; input arrives at the window.
class SimRendererWidget : public QOpenGLWindow,
                          protected QOpenGLExtraFunctions {
  Q_OBJECT
public:
  explicit SimRendererWidget(QWindow *parent = nullptr);
  ~SimRendererWidget() override;

  // Overlay painted over the 3D frame by the renderer (QPainter on the GL
  // window): `w` is never shown; each frame it is rendered at the window's size
  // into an image that is composited on top. Not owned. Used for the FPV
  // telemetry HUD, which used to be a child widget over the view.
  // The widget's `changed()` signal (if it has one) marks the cached image
  // stale; without it the HUD is re-rendered every frame.
  void setHudWidget(QWidget *w);
  void setHudVisible(bool on);
  // The HUD image is cached and re-rendered only when the HUD says it changed,
  // the window size or pixel ratio changed, or it was just shown. Off = render
  // every frame (what it used to do); for measuring.
  void setHudCache(bool on) { hudCache_ = on; }
  int hudRenderCount() const { return hudRenders_; }
  qint64 hudPaintNs() const { return hudNs_; } // total time inside paintHud
  void resetHudStats() { hudRenders_ = 0; hudNs_ = 0; }
public slots:
  void invalidateHud(); // the HUD content changed: re-render before next paint
public:

  // Chunk read-back on a context reset uses glGetBufferSubData, which GLES
  // lacks; there (or when forced here, for tests) a CPU copy of each chunk is
  // kept instead. Set before chunks are loaded.
  void setForceChunkCpuCopy(bool on) { forceChunkCpu_ = on; }
  // Read-backs done so far by any renderer (a teardown must do none).
  static int chunkReadBacks();

  // A screenshot of the view, HUD included: schedules a frame, waits (a nested
  // event loop, at most `timeoutMs`) until it has been presented, then reads it
  // back with QOpenGLWindow::grabFramebuffer. grabFramebuffer alone only reads
  // whatever the framebuffer holds, which after a swap need not be the latest
  // frame. Null image if the window is not exposed. This replaces
  // QWidget::grab(), which cannot capture a native GL window.
  QImage grabFrame(int timeoutMs = 2000);

public slots:
  void setSnapshot(const vsim::SimSnapshot &s);

public:
  // Replace the procedural box with an imported airframe mesh (triangle
  // soup + per-vertex normals, body frame, meters). Upload is deferred
  // to the next paintGL so this is safe to call from the UI thread.
  // Empty positions reverts to the procedural body.
  void setDroneMesh(const std::vector<QVector3D> &positions,
                    const std::vector<QVector3D> &normals);
  // Editable motor markers: body-frame position, unit thrust axis, and
  // spin (+1 CCW / -1 CW) per rotor. Drawn over the body.
  void setMotorLayout(const std::array<QVector3D, 4> &pos,
                      const std::array<QVector3D, 4> &axis,
                      const std::array<int, 4> &spin);
  // Which airframe the procedural model depicts (preset id such as "5in", or
  // empty for a custom vehicle: classed by prop radius) and its prop radius [m].
  // The model is sized from this and the motor layout; an imported mesh
  // (setDroneMesh) replaces it.
  void setAirframeStyle(const QString &presetId, float propRadius);
  // Body-frame center-of-mass marker (crosshair); diagnostic for how far
  // the modeled origin sits from the true CoM.
  void setComMarker(const QVector3D &com);

  // Static world obstacles (boxes/spheres/cylinders) in the NED world frame.
  // Stored + redrawn each frame; safe to call from the UI thread.
  void setObstacles(const QVector<vsim::Obstacle> &obs);

  // Helipad landing platforms: cylinder pads marked "H", scattered on flat
  // ground. Each entry is (worldX, worldY, terrainHeight) — terrainHeight is the
  // positive elevation. The pad is a tall cylinder whose base is BURIED below the
  // surface and whose deck sits kHelipadDeckM ABOVE it, so the deck reliably
  // clears undulating terrain (a thin flush pad gets swallowed by the ground).
  static constexpr float kHelipadRadiusM = 3.0f;
  static constexpr float kHelipadDeckM = 0.90f; // deck height above terrain
  static constexpr float kHelipadHeightM =
      2.20f; // total cylinder (deck + skirt)
  void setHelipads(const std::vector<QVector3D> &pads);

  // Far clip [m]. Fog (see drawLit) has fully taken over well before this; the
  // old 200 m clip chopped the terrain off in a hard line against the sky.
  static constexpr float kFarPlaneM = 3000.0f;
  // Near clip [m]. A 24-bit depth buffer resolves about d^2 / (near * 2^24) at
  // distance d, so 0.05 m against a 3000 m far plane could not tell surfaces
  // 5 cm apart apart beyond ~200 m (horizon plane vs arena, decals). The chase
  // camera never needs to see closer than this; FPV and the belly camera do
  // (the lens sits near the ground when landed), so they keep the small one.
  static constexpr float kNearChaseM = 0.25f;
  static constexpr float kNearLensM = 0.05f;
  // Cylinders at or below this radius [m] are drawn as banded survey poles.
  static constexpr float kPoleMaxR = 0.12f;
  // True for a finite procedural arena: its terrain gets the ground detail and
  // a detailed ground plane continues it to the horizon.
  void setHorizonGround(bool on) {
    horizonOn_ = on;
    update();
  }

  // Training course: glowing halo gates to fly through. setTrainingGates
  // replaces the gate layout; setTrainingActive updates which gate to aim for
  // (highlighted) and whether to draw the guidance arrow (drone -> next gate).
  // Empty gates clears the course.
  void setTrainingGates(const QVector<vsim::RingGate> &gates);
  void setTrainingActive(int activeIndex, bool showArrow);

  // Imported static world mesh (triangle soup + normals, NED world frame).
  // Deferred upload like the drone mesh; empty positions clears it. `colors`
  // is optional per-vertex RGB (baked from the source materials) — pass {} to
  // fall back to neutral grey.
  void setWorldMesh(const std::vector<QVector3D> &positions,
                    const std::vector<QVector3D> &normals,
                    const std::vector<QVector3D> &colors = {});

  // Streaming world chunks (endless procedural terrain). Each chunk is an
  // independently uploaded colored mesh keyed by a packed (cx,cy). The streamer
  // adds/removes chunks as the drone moves; uploads are deferred to the next
  // paintGL like the single world mesh. Coexists with obstacles/training; an
  // imported world mesh and chunks are mutually exclusive in practice.
  void setWorldChunk(qint64 key, const std::vector<QVector3D> &positions,
                     const std::vector<QVector3D> &normals,
                     const std::vector<QVector3D> &colors);
  void removeWorldChunk(qint64 key);
  void clearWorldChunks();
  bool hasWorldChunks() const {
    return !worldChunks_.empty() || !pendingChunkUploads_.empty();
  }

  // Per-chunk instanced flora. `interleaved` is `count` blades of 9 floats each:
  // [posx,posy,posz, yaw,height,flower, tintr,tintg,tintb]. Deferred upload like
  // the chunk meshes; keyed the same so it loads/unloads with its chunk.
  // Grass OFF (default): these are inline no-ops so callers need no #ifdef.
#ifdef VAYU_SIM_GRASS
  void setChunkFlora(qint64 key, const std::vector<float> &interleaved,
                     int count, float centerX, float centerY, float halfExtent);
  void removeChunkFlora(qint64 key);
  void clearChunkFlora();
  void setFloraVisible(bool on) {
    floraVisible_ = on;
    update();
  }

  // GPU-driven grass (regenerated on the GPU each frame). Available only on a
  // GL 4.3+ context; falls back to the CPU chunk flora otherwise.
  bool gpuGrassReady() const { return gpuGrass_.ready(); }
  void setGpuGrassParams(const vsim::GpuGrass::Params &p) {
    gpuGrass_.setParams(p);
  }
  void setGpuGrassActive(bool on) {
    gpuGrassActive_ = on;
    update();
  }
#else
  void setChunkFlora(qint64, const std::vector<float> &, int, float, float,
                     float) {}
  void removeChunkFlora(qint64) {}
  void clearChunkFlora() {}
  void setFloraVisible(bool) {}
  bool gpuGrassReady() const { return false; }
  void setGpuGrassActive(bool) {}
  // setGpuGrassParams omitted: its signature needs GpuGrass::Params (a guarded
  // type). Callers guard the call under VAYU_SIM_GRASS.
#endif
  // Terrain lighting look knobs — kept in sync with the grass GrassLook so the
  // ground and grass warm/brighten together. Applied live in drawLit.
  void setTerrainLook(float sunIntensity, float ambientStrength) {
    litSunInt_ = sunIntensity;
    litAmbStr_ = ambientStrength;
    update();
  }
  // World-space XY the terrain streamer should centre on: the free-fly camera
  // when roaming (sim stopped), otherwise the drone. Lets endless terrain follow
  // both WASD navigation and actual flight.
  QVector3D streamCenter() const { return freeFly_ ? camPos_ : snap_.pos_w; }
  // Where the vehicle is drawn (the last snapshot), whatever the camera does.
  QVector3D vehiclePos() const { return snap_.pos_w; }
  // NED heading (radians, 0 = north) the minimap should orient by: the free-fly
  // look direction when roaming, otherwise the drone's body heading.
  float viewHeadingRad() const;

  // Enable Blender-style obstacle gizmo editing (click-select, G move /
  // R rotate / S scale, X/Y/Z constrain) — World mode while the sim is
  // stopped. Mutually exclusive with motor editing.
  void setObstacleEditMode(bool on);
  void selectObstacle(int index); // drive selection from the list

  // Onboard FPV camera: ride the drone, looking forward along body +X (the
  // body mesh is hidden so it doesn't fill the lens). Off = orbit camera.
  void setFpv(bool on) {
    fpv_ = on;
    update();
  }
  bool fpv() const { return fpv_; }

  // Bird's-eye "down-cam": sit a fixed height above the drone and look straight
  // down (NED +Z), north up. Used by the Down-Cam PiP. Takes precedence over
  // fpv/orbit while on.
  void setDownCam(bool on) {
    downCam_ = on;
    update();
  }
  bool downCam() const { return downCam_; }

  // Free-fly camera: WASD/QE to move, left-drag to look, NOT locked to the
  // drone. Used in World mode while the sim is stopped so you can roam the
  // imported world. Off = the camera orbits/follows the drone (3rd person).
  // Enabling seeds the free camera at the current orbit eye for a smooth swap.
  void setFreeFly(bool on);
  bool freeFly() const { return freeFly_; }

  // Show the imported world mesh + obstacles. Off in Vehicle mode (just the
  // airframe); on in World mode (the whole world).
  void setWorldVisible(bool on);

  // Enable Blender-style motor gizmo editing (click-select, G move /
  // R rotate, X/Y/Z constrain). Only meaningful when the sim is stopped;
  // the SimulatorWidget toggles this on stop / off on start.
  void setMotorsEditable(bool on);

signals:
  // Emitted on gizmo confirm. pos is CoM-frame (same frame as
  // setMotorLayout); axis is the unit thrust axis. index in [0,3].
  void motorEdited(int index, QVector3D posComFrame, QVector3D axis);
  void motorSelected(int index);
  // Obstacle gizmo: new pos/size/rotate (world frame) on confirm; selection.
  void obstacleEdited(int index, QVector3D pos, QVector3D size,
                      QVector3D rotate);
  void obstacleSelected(int index);

public:
  // ---- GL context lifetime (see initResources / releaseGL) ----
  // Number of GL objects (buffers, VAOs, programs, textures, FBOs) this view
  // currently holds. Zero before the first init and after the context is torn
  // down; used by tests to prove nothing leaks across a context recreation.
  int liveGlObjects() const;
  // How many times initResources() has built the GL side (1 per context).
  int glInitCount() const { return glInitCount_; }
  int contextGoneCount() const { return goneCount_; } // aboutToBeDestroyed seen
  bool contextWasCurrentWhenGone() const { return goneCurrent_; }
  // Drops every GL object and builds them all again in the CURRENT context,
  // exactly what a context recreation does; for tests and tools that want the
  // re-init path without moving a window.
  void resetGlResources();
  // Count GL errors (glGetError) at the end of every frame into glErrorCount().
  void setGlErrorCheck(bool on) { glErrorCheck_ = on; }
  int glErrorCount() const { return glErrors_; }

protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;

  // Input arrives at the native window (the container forwards focus to it).
  void mousePressEvent(QMouseEvent *e) override;
  void mouseMoveEvent(QMouseEvent *e) override;
  void wheelEvent(QWheelEvent *e) override;
  void keyPressEvent(QKeyEvent *e) override;

private:
  // Builds every program and static mesh. Idempotent: safe to run again on a
  // new context (it first drops whatever a previous context left behind).
  void initResources();
  // Frees every GL object with the context current. With readBack, re-queues
  // what must be re-uploaded into a context that follows (chunks are read back
  // from the GPU, or taken from their CPU copy on GLES); without it (teardown:
  // destructor, init after the old context was lost) nothing is read, only
  // released. Callers: onContextGone (true: the renderer survives and the data
  // is still needed), resetGlResources (true: same), initResources (false:
  // stale handles), destructor (false: nothing will use the data).
  void releaseGLResources(bool readBack);
  void onContextGone(); // aboutToBeDestroyed slot: makeCurrent + release, reading back
  bool chunkCpuMode() const; // keep CPU copies of chunks (GLES / forced)
  void eraseChunk(qint64 key); // release + erase (a Mesh dtor frees no GL)
  void eraseAllChunks();
#ifdef VAYU_SIM_GRASS
  void eraseFlora(qint64 key);
  void eraseAllFlora();
#endif
  // -- GL helpers --
  struct Mesh {
    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer vbo{QOpenGLBuffer::VertexBuffer};
    int vertex_count = 0;
    GLenum primitive = GL_TRIANGLES;
    // Frees the GL objects (context must be current) and returns the mesh to
    // its never-uploaded state, so a later build/upload recreates them.
    void release() {
      vbo.destroy();
      vao.destroy();
      vertex_count = 0;
    }
    int liveObjects() const {
      return (vao.isCreated() ? 1 : 0) + (vbo.isCreated() ? 1 : 0);
    }
  };
  template <class F> void forEachMesh(F &&f);

  void buildGroundGrid();
  void buildObstacleMeshes(); // unit box / sphere / cylinder (pos+normal)
  void buildHelipadMeshes();  // unit pad platform + "H"/ring mark (pos+normal)
  void buildAxes();
  void buildDroneBody();
  void buildRotorDisk();
  void buildThrustLine();
  void buildComMarker();
  void buildGizmo();  // unit +X arrow + unit XY ring for the gizmo
  void buildDigits(); // line-stroke glyphs for motor numbers 1..4
  void drawMotorLabels(const QMatrix4x4 &view); // billboarded numbers
  void buildNorth();                            // body +X arrow + "N" glyph
  void drawNorthIndicator(const QMatrix4x4 &view);
  void buildRing();       // unit torus (major R=1, in local XY plane)
  void buildGuideArrow(); // unit solid arrow along +X (shaft + cone head)
  void drawTraining(const QMatrix4x4 &view); // halo gates + guidance arrow
  void uploadDroneMesh(); // flushes pending_* into droneMesh_ (GL-current)
  void uploadWorldMesh(); // flushes pendingWorld_* into worldMesh_
  void
  flushChunkUpdates(); // applies queued chunk uploads/removals (GL-current)
#ifdef VAYU_SIM_GRASS
  void buildGrassBlade(); // shared unit-blade geometry (crossed tapered quads)
#endif
  void
  buildShadowMap(); // create the directional shadow-map FBO + depth texture
  void renderShadowPass(bool showWorld,
                        bool showChunks); // terrain depth from the sun
#ifdef VAYU_SIM_GRASS
  void
  flushFloraUpdates(); // applies queued flora uploads/removals (GL-current)
  void drawFlora(const QMatrix4x4 &view); // instanced blades over the chunks
#endif
  // Upload an interleaved [px,py,pz,nx,ny,nz] array into a lit-shader mesh.
  void uploadLitMesh(Mesh &m, const std::vector<float> &interleaved);
  // As uploadLitMesh, but the array is [px,py,pz,nx,ny,nz,r,g,b] and vertex
  // colors (attribute 2) are enabled — used for the colored world mesh.
  void uploadColoredMesh(Mesh &m, const std::vector<float> &interleaved);

  // ---- gizmo editing (Blender-style; active only when editable_) ----
  enum class Tool { None, Move, Rotate, Scale };
  QMatrix4x4 modelMatrix() const; // body -> world
  void cameraEyeTarget(QVector3D *eye, QVector3D *target) const;
  void rayThroughPixel(const QPoint &px, QVector3D *o, QVector3D *d) const;
  bool pickMotor(const QPoint &px, int *outIndex) const;
  void beginTool(Tool t);
  void updateToolFromMouse(const QPoint &px);
  void commitTool(bool confirm);
  void drawGizmo(const QMatrix4x4 &view);

  // ---- obstacle gizmo editing (active only when obsMode_) ----
  bool pickObstacle(const QPoint &px, int *outIndex) const;
  void beginObsTool(Tool t);
  void updateObsToolFromMouse(const QPoint &px);
  void commitObsTool(bool confirm);
  void drawObsGizmo(const QMatrix4x4 &view);

  void drawMesh(const Mesh &m, const QMatrix4x4 &model, const QVector3D &color);
  // Draws a pos+normal mesh through the lit program. view and model are
  // kept separate so normals transform by mat3(model) in world space.
  void resetGlState();  // known GL state at the start of a frame (QPainter dirties it)
  void paintHud();      // composites the HUD widget over the finished frame
  void drawLit(const Mesh &m, const QMatrix4x4 &view, const QMatrix4x4 &model,
               const QVector3D &color, bool terrain = false, float bands = 0.0f);

  // Tree crown as lumpy blobs / conifer tiers inside the obstacle's sphere.
  void drawCrown(const QMatrix4x4 &view, const vsim::Obstacle &o,
                 const QVector3D &col);

  QMatrix4x4 cameraView() const;
  QVector3D freeForward() const; // free-cam look direction from yaw/pitch
  float bodyYawRad() const;      // drone heading (yaw about world Z, NED)
  // Third-person orbit eye offset from the drone. The azimuth is locked to the
  // drone's heading (cam_yaw_ is the offset RELATIVE to the body, changed only
  // by mouse drag) so the camera yaws with the vehicle and stays easy to track.
  QVector3D orbitOffset() const;
  bool freeFlyMove(int key, bool fast); // WASD/QE → move camPos_; true if used

  // Flat shader: vec3 position + uniform color + MVP (grid/axes/markers).
  std::unique_ptr<QOpenGLShaderProgram> prog_;
  int u_mvp_ = -1;
  int u_color_ = -1;

  // Lit shader: position + normal, directional Lambert + ambient, plus aerial
  // perspective (distance fog into the sky) and an ACES tonemap.
  std::unique_ptr<QOpenGLShaderProgram> progLit_;
  int ul_mvp_ = -1;
  int ul_model_ = -1;
  int ul_nmat_ = -1;
  int ul_color_ = -1;
  int ul_sundir_ = -1;
  int ul_campos_ = -1;
  int ul_fogdensity_ = -1;
  int ul_fogstart_ = -1;
  int ul_lightvp_ = -1;
  int ul_shadowtex_ = -1;
  int ul_shadowon_ = -1;
  int ul_drone_ = -1;
  int ul_droner_ = -1;
  QVector3D droneShadowPos_;  // drone position for the analytic shadow
  float droneShadowR_ = 0.0f; // shadow radius [m]; 0 = off
  int ul_sunint_ = -1;
  int ul_ambstr_ = -1;
  int ul_terrain_ = -1;
  int ul_bands_ = -1;
  float litSunInt_ = 1.0f; // terrain lighting look knobs (match grass)
  float litAmbStr_ = 1.0f;

  // Shadow map: a directional depth buffer rendered from the sun's view each
  // frame (terrain casters); lit terrain + grass sample it to drop into shade.
  std::unique_ptr<QOpenGLShaderProgram> progDepth_; // depth-only, light-space
  int ud_lightmvp_ = -1;
  unsigned int shadowFbo_ = 0;
  unsigned int shadowTex_ = 0;
  int shadowSize_ = 2048;
  bool shadowReady_ = false; // FBO built ok
  bool shadowOn_ = false;    // a shadow map was rendered this frame
  QMatrix4x4 lightVP_;       // light view-projection (world -> light clip)
  int fbW_ = 1, fbH_ = 1;    // saved framebuffer size (viewport restore)

  // Sky shader: attribute-less fullscreen triangle, view-ray gradient + glow.
  std::unique_ptr<QOpenGLShaderProgram> progSky_;
  int us_invvp_ = -1;
  int us_sundir_ = -1;
  int us_time_ = -1;
  QOpenGLVertexArrayObject skyVao_;

  QPointer<QWidget> hud_; // painted over the frame (see setHudWidget)
  bool hudVisible_ = false;
  bool hudCache_ = true;
  bool hudDirty_ = true;
  bool hudHasSignal_ = false;
  QImage hudImg_;
  QSize hudSize_;
  qreal hudDpr_ = 0;
  int hudRenders_ = 0;
  qint64 hudNs_ = 0;
  bool forceChunkCpu_ = false;
  // CPU copies of the loaded chunks, kept only where VBO read-back is not
  // available (GLES) or forced.
  std::map<qint64, std::vector<float>> chunkCpu_;

  // GL context lifetime bookkeeping.
  bool glLive_ = false; // resources exist in the current context
  QOpenGLContext *glCtx_ = nullptr;
  int glInitCount_ = 0;
  int goneCount_ = 0;
  bool goneCurrent_ = false;
  bool glErrorCheck_ = false;
  int glErrors_ = 0;

  // Animation clock (advances per paint); drives the sky glow AND, when built,
  // the grass wind — kept general so it survives with grass compiled out.
  float floraTime_ = 0.0f;

#ifdef VAYU_SIM_GRASS
  // Flora shader: instanced grass/flower blades, wind + distance fade + fog.
  std::unique_ptr<QOpenGLShaderProgram> progFlora_;
  int uf_vp_ = -1, uf_campos_ = -1, uf_time_ = -1;
  int uf_fadestart_ = -1, uf_fadeend_ = -1, uf_sundir_ = -1;
  int uf_fogdensity_ = -1, uf_fogstart_ = -1;
  QOpenGLBuffer grassVbo_[3]; // shared blade geometry at 3 LODs (24/12/6 verts)
  int grassVerts_[3] = {0, 0, 0};
  QOpenGLVertexArrayObject
      floraVao_; // shared; geometry+instance bound per draw
  bool floraVisible_ = true;

  // GPU grass (compute-generated each frame). Inactive unless the endless biome
  // selects it and the context supports compute.
  GpuGrass gpuGrass_;
  bool gpuGrassActive_ = false;
#endif

  // Camera world position (NED), refreshed each paintGL; fed to the lit shader
  // for distance fog.
  QVector3D camEye_;
  // Sun direction (toward the light, world NED) shared by lit + sky. A real
  // directional key ~45 deg above the horizon (z<0 is up in NED) for moody,
  // directional overcast light rather than flat noon overhead.
  QVector3D sunDir_{0.55f, 0.42f, -0.72f};

  Mesh ground_;
  Mesh unitBox_;     // [-0.5,0.5]^3, pos+normal (lit) — scaled per obstacle
  Mesh unitSphere_;  // radius-1 UV sphere, pos+normal
  Mesh horizonQuad_; // unit XY quad (normal up): ground out to the horizon
  bool horizonOn_ = false;
  float horizonZ_ = 0.0f; // NED z of the horizon ground under an arena
  QVector3D horizonCol_{0.33f, 0.45f, 0.24f};
  Mesh unitCyl_;     // radius-1, height-1 cylinder (+caps), pos+normal
  Mesh helipadDisk_; // radius-1 pad platform, local z in [-1(top)..0(base)]
  Mesh helipadMark_; // flat "H" + ring on the pad top (white), pos+normal
  std::vector<QVector3D> helipads_; // (x, y, terrainHeight) per pad
  QVector<vsim::Obstacle> obstacles_;
  Mesh axes_;
  Mesh body_;
  // Procedural airframe (DroneModel): one body mesh + a prop mesh per motor,
  // rebuilt lazily when the layout/style changes, spun per real motor speed.
  void rebuildDroneModel();
  void updateRotors(float dt);
  void drawDroneModel(const QMatrix4x4 &view, const QMatrix4x4 &model);
  void drawDroneLocator(const QMatrix4x4 &view, const QMatrix4x4 &model);
  Mesh modelBody_;
  std::array<Mesh, 4> modelProp_;
  vsim::DroneModel model_;
  bool modelDirty_ = true;
  QString presetId_ = QStringLiteral("5in");
  float propRadius_ = 0.0622f;
  std::array<float, 4> rotorAngle_ = {0, 0, 0, 0}; // accumulated [rad]
  std::array<float, 4> rotorBlur_ = {0, 0, 0, 0};  // blur-disc alpha 0..1
  QElapsedTimer rotorClock_;
  Mesh rotor_;
  Mesh thrustLine_;   // unit segment along +Z, scaled per motor axis
  Mesh comMarker_;    // small 3-axis crosshair
  Mesh gizmoArrow_;   // unit segment along +X (rotated per axis)
  Mesh gizmoRing_;    // unit circle in XY plane (rotated per axis)
  Mesh digitMesh_[4]; // glyphs for "1".."4", billboarded over each motor
  Mesh locator_;      // screen-space heading chevron (small-model locator)
  Mesh northArrow_;   // body +X arrow (shaft + head), rotates with the body
  Mesh glyphN_;       // "N" glyph billboarded at the arrow tip
  Mesh ring_;         // unit torus for training halo gates (lit, pos+normal)
  Mesh guideArrow_;   // unit solid arrow (lit) pointing drone -> next gate

  // Training course state (gameplay overlay; populated only in training mode).
  QVector<vsim::RingGate> gates_;
  int trainActive_ = 0;     // index of the gate to aim for next
  bool trainArrow_ = false; // draw the guidance arrow this frame
  float glowPhase_ = 0.0f;  // advances per paint for the gate pulse

  // Imported mesh (pos+normal interleaved). hasMesh_ gates body vs mesh.
  Mesh droneMesh_;
  bool hasMesh_ = false;
  bool meshDirty_ = false;
  // CPU copies, kept for the life of the view: a new GL context re-uploads
  // from them (meshDirty_ is raised again by releaseGLResources).
  std::vector<QVector3D> pendingPos_;
  std::vector<QVector3D> pendingNrm_;

  // Imported world mesh (pos+normal interleaved; same deferred-upload path).
  Mesh worldMesh_;
  bool hasWorldMesh_ = false;
  bool worldMeshDirty_ = false;
  // CPU copies, kept (see pendingPos_) so a new context can re-upload them.
  std::vector<QVector3D> pendingWorldPos_;
  std::vector<QVector3D> pendingWorldNrm_;
  std::vector<QVector3D> pendingWorldCol_; // per-vertex RGB (may be empty)

  // Streaming terrain chunks. Heap-allocated Meshes so addresses stay stable in
  // the map (and GL handles aren't moved). Uploads/removals are queued from the
  // UI thread and flushed in paintGL where the GL context is current.
  std::map<qint64, std::unique_ptr<Mesh>> worldChunks_;
  struct PendingChunk {
    qint64 key;
    std::vector<float> data;
  }; // 9 floats/vert
  std::vector<PendingChunk> pendingChunkUploads_;
  std::vector<qint64> pendingChunkRemovals_;
  bool chunksDirty_ = false;
  bool clearAllChunks_ = false;

#ifdef VAYU_SIM_GRASS
  // Per-chunk instanced flora (grass/flowers), keyed like worldChunks_. Each
  // holds a VAO binding the shared blade geometry + this chunk's instance VBO.
  struct FloraChunk {
    QOpenGLBuffer inst{QOpenGLBuffer::VertexBuffer};
    int count = 0;
    float cx = 0.0f, cy = 0.0f, half = 0.0f; // chunk centre + half-extent (LOD)
  };
  std::map<qint64, std::unique_ptr<FloraChunk>> floraChunks_;
  struct PendingFlora {
    qint64 key;
    std::vector<float> data;
    int count;
    float cx, cy, half;
  };
  std::map<qint64, PendingFlora> floraCpu_; // as chunkCpu_, for flora
  std::vector<PendingFlora> pendingFloraUploads_;
  std::vector<qint64> pendingFloraRemovals_;
  bool floraDirty_ = false;
  bool clearAllFlora_ = false;
#endif

  // Editable motor layout (defaults mirror the firmware quad geometry).
  std::array<QVector3D, 4> motorPos_ = {
      QVector3D(0.13f, +0.22f, 0.0f), QVector3D(-0.13f, +0.20f, 0.0f),
      QVector3D(-0.13f, -0.20f, 0.0f), QVector3D(0.13f, -0.22f, 0.0f)};
  std::array<QVector3D, 4> motorAxis_ = {
      QVector3D(0, 0, -1), QVector3D(0, 0, -1), QVector3D(0, 0, -1),
      QVector3D(0, 0, -1)};
  std::array<int, 4> motorSpin_ = {+1, -1, +1, -1};
  QVector3D comOffset_{0, 0, 0};

  // Gizmo edit state.
  bool editable_ = true; // toggled by SimulatorWidget (sim stopped)
  int selected_ = -1;    // selected motor index, -1 = none
  Tool tool_ = Tool::None;
  int axisLock_ = -1; // -1 = view-plane / default, 0/1/2 = X/Y/Z
  QPoint toolStartMouse_;
  QVector3D startPos_;   // CoM-frame motor pos at tool start
  QVector3D startAxis_;  // unit thrust axis at tool start
  QVector3D startWorld_; // world motor pos at tool start
  QVector3D planeHit0_;  // world drag-plane hit at tool start

  // Obstacle gizmo edit state (World mode).
  bool obsMode_ = false;
  int selObs_ = -1;
  Tool obsTool_ = Tool::None;
  int obsAxis_ = -1;
  QPoint obsStartMouse_;
  QVector3D obsStartPos_, obsStartSize_, obsStartRot_;
  QVector3D obsPlaneHit0_;

  // Camera state (orbit around drone).
  float cam_radius_ = 4.0f;
  float cam_yaw_ = 0.7f;   // orbit azimuth RELATIVE to the drone heading
  float cam_pitch_ = 0.5f; // tilt; >0 = eye above ground looking down (NED)
  bool fpv_ = false;       // onboard FPV camera vs orbit
  bool downCam_ = false;   // belly-mounted downward camera (Down-Cam PiP)
  float downCamOffset_ = 0.05f; // metres below the CoM the belly lens sits
  bool freeFly_ = false;     // WASD free-roam camera (sim stopped, World mode)
  bool worldVisible_ = true; // draw world mesh + obstacles (World mode)
  QVector3D camPos_{-4.0f, -4.0f, -3.0f}; // free-cam world position (NED)
  QPoint last_mouse_;

  QMatrix4x4 proj_;
  float nearCur_ = 0.0f; // near plane proj_ was built with
  void updateProjection(); // rebuild proj_ if the near plane for the mode changed
  vsim::SimSnapshot snap_;
};

} // namespace vsim
