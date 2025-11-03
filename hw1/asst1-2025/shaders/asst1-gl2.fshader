uniform float uTime;
uniform sampler2D uTexUnit0;

varying vec2 vTexCoord0;
varying vec3 vColor;
varying vec2 vFragPos;

// Heart shape distance function
float sdHeart( in vec2 p )
{
    p.x = abs(p.x);

    if( p.y+p.x>1.0 )
        return sqrt(dot(p-vec2(0.25,0.75), p-vec2(0.25,0.75))) - sqrt(2.0)/4.0;
    return sqrt(min(dot(p-vec2(0.00,1.00), p-vec2(0.00,1.00)),
                    dot(p-0.5*max(p.x+p.y,0.0), p-0.5*max(p.x+p.y,0.0)))) * sign(p.x-p.y);
}

void main(void) {
  vec4 texColor0 = texture2D(uTexUnit0, vTexCoord0);

  vec3 guid_color = vec3(0.5, 0.4, 0.3); // set the guide color
  vec2 spliced_pos = vFragPos;
  float r;
  for (int i=0; i<2; i++){
    spliced_pos = fract(spliced_pos * 2.0);
    spliced_pos.x -= 0.5;
    r = sdHeart(spliced_pos);
    r = abs(sin(r * 15.0 + uTime * 7.0) / 15.0);
    r = 0.02 / r;
    guid_color += guid_color * r;
  }
  gl_FragColor = vec4(guid_color, 1) * 0.2 + texColor0 * 0.8;
}