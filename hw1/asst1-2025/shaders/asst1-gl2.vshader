uniform float uVertexScale;

// ass1 uniform append
uniform float uWindowSizeX, uWindowSizeY;

attribute vec2 aPosition;
attribute vec3 aColor;
attribute vec2 aTexCoord0;

varying vec3 vColor;
varying vec2 vFragPos;
varying vec2 vTexCoord0;

void main() {
  gl_Position = vec4(aPosition.x * uVertexScale * uWindowSizeX, aPosition.y * uWindowSizeY, 0,1);
  vColor = aColor;
  vFragPos = aPosition.xy;
  vTexCoord0 = aTexCoord0;
}
