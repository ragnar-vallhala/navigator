// The GL side of the terrain material (see TerrainMaterial.h for the rule):
// the GLSL text and the photo textures.
#pragma once

#include "TerrainMaterial.h"

#include <QByteArray>
#include <QOpenGLShaderProgram>
#include <QtGui/qopengl.h>

class QOpenGLExtraFunctions;

namespace vsim::terrain {

// The GLSL to append after the lit program's shared GLSL and before its main
// shader: the uniforms u_t0..u_t2, u_n0..u_n2, u_matq and terrainMaterial().
QByteArray terrainMaterialGLSL();

// The six photo textures (grass, rock, dirt; diffuse and normal) from the Qt
// resources, uploaded on first load(). One per GL context.
class TerrainTextures {
public:
  TerrainTextures() = default;
  ~TerrainTextures();
  TerrainTextures(const TerrainTextures &) = delete;
  TerrainTextures &operator=(const TerrainTextures &) = delete;

  bool load(QOpenGLExtraFunctions *gl);
  bool loaded() const { return tex_[0] != 0; }
  // contextCurrent: the owning context is current, so the objects can be
  // deleted; otherwise they are just forgotten (the context took them).
  void release(QOpenGLExtraFunctions *gl, bool contextCurrent);
  // Binds the six textures to units 2..7 and sets the sampler uniforms.
  void bind(QOpenGLExtraFunctions *gl, QOpenGLShaderProgram *prog) const;

private:
  GLuint tex_[6] = {};
};

} // namespace vsim::terrain
