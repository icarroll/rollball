#version 330 core

// Image shaders implement the mainImage() function in order to generate the procedural images by computing a color for each pixel. This function is expected to be called once per pixel, and it is responsability of the host application to provide the right inputs to it and get the output color from it and assign it to the screen pixel. The prototype is:
//
// void mainImage( out vec4 fragColor, in vec2 fragCoord );
//
// where fragCoord contains the pixel coordinates for which the shader needs to compute a color. The coordinates are in pixel units, ranging from 0.5 to resolution-0.5, over the rendering surface, where the resolution is passed to the shader through the iResolution uniform (see below).
//
// The resulting color is gathered in fragColor as a four component vector, the last of which is ignored by the client. The result is gathered as an "out" variable in prevision of future addition of multiple render targets.
//
// -----
//
// Shader can be fed with different types of per-frame static information by using the following uniform variables:
//
// uniform vec3 iResolution;
// uniform float iTime;
// uniform vec4 iMouse;
// uniform samplerXX iChanneli;
//
// //uniform float iTimeDelta;
// //uniform float iFrame;
// //uniform float iChannelTime[4];
// //uniform vec4 iDate;
// //uniform float iSampleRate;
// //uniform vec3 iChannelResolution[4];

// License Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
// Created by S. Guillitte 2015
// Modified by Isaac Carroll 2019

out vec4 FragColor;
//in vec3 FragPos;
in vec2 uv;

// ignored?
//in vec3 Normal;
//uniform vec3 lightPos;
//uniform vec3 lightColor;
//uniform vec3 objectColor;

vec3 iResolution = vec3(1.0, 1.0, 1.0);

vec3 envcolor = vec3(0.2, 0.3, 0.3);

//uniform float iTime;
float iTime = 0.0;

uniform mat3 obj_rotation;
uniform vec3 camera_look;
uniform vec3 camera_right;
uniform vec3 camera_up;

//float zoom=1.;
float size = 1.0; //TODO doesn't actually work properly
float zoom=sqrt(4*4+4*4+4*4)/size;

vec2 cmul( vec2 a, vec2 b )  { return vec2( a.x*b.x - a.y*b.y, a.x*b.y + a.y*b.x ); }
vec2 csqr( vec2 a )  { return vec2( a.x*a.x - a.y*a.y, 2.*a.x*a.y  ); }

mat2 rot(float a) {
    return mat2(cos(a),sin(a),-sin(a),cos(a));
}

vec2 iSphere( in vec3 ray_origin, in vec3 ray_dir, in vec4 sph )//from iq
{
    vec3 oc = ray_origin - sph.xyz;
    float b = dot( oc, ray_dir );
    float c = dot( oc, oc ) - sph.w*sph.w;
    float h = b*b - c;
    if( h<0.0 ) return vec2(-1.0);
    h = sqrt(h);
    return vec2(-b-h, -b+h );
}

float map(in vec3 p) {
    float res = 0.;

    vec3 c = p;
    for (int i = 0; i < 10; ++i) {
        p =.7*abs(p+cos(iTime*0.15+1.6)*0.15)/dot(p,p) -.7+cos(iTime*0.15)*0.15;
        p.yz= csqr(p.yz);
        p=p.zxy;
        res += exp(-19. * abs(dot(p,c)));

    }
    return res/2.;
}

vec3 raymarch( in vec3 ray_origin, vec3 ray_dir, vec2 tminmax )
{
    float t = tminmax.x;
    float dt = .1;
    //float dt = .1 - .075*cos(iTime);//animated
    vec3 col= vec3(0.);
    float c = 0.;
    for( int i=0; i<64; i++ )
    {
        t+=dt*exp(-2.*c);
        if(t>tminmax.y)break;

        vec3 pos = ray_origin+t*ray_dir;
        c = map(ray_origin+t*ray_dir);

        //col = .99*col+ .08*vec3(c*c, c, c*c*c);//green
        col = .99*col+ .08*vec3(c*c*c, c*c, c);//blue
    }
    return col;
}

void mainImage( out vec4 fragColor, in vec2 fragCoord )
{
    float time = iTime;
    vec2 q = fragCoord.xy / iResolution.xy;
    vec2 p = -1.0 + 2.0 * q;
    p.x *= iResolution.x/iResolution.y;
    vec2 m = vec2(0.);
    m-=.5;

    // camera
    vec3 ray_origin = zoom * camera_look;
    ray_origin = obj_rotation * ray_origin;
    vec3 center = vec3( 0.0 , 0.0, 0.0 );
    vec3 uu = obj_rotation * camera_right;
    vec3 vv = obj_rotation * camera_up;
    vec3 ww = normalize(center - ray_origin);
    //vec3 ray_dir = normalize( p.x*uu + p.y*vv + 4.0*ww )*0.975;
    vec3 ray_dir = normalize( p.x*uu + p.y*vv + 4.0*ww )*0.985;

    vec2 tmm = iSphere( ray_origin, ray_dir, vec4(0.,0.,0.,2.) );

    // raymarch
    vec3 col = raymarch(ray_origin,ray_dir,tmm);
    //if (tmm.x<0.)col = texture(iChannel0, ray_dir).rgb;
    /*
    // temp: draw border of the quad
    if (tmm.x<0.) {
        if (q.x < 0.01 || q.x > 0.99 || q.y < 0.01 || q.y > 0.99) {}
        else discard;
    }
    */
    if (tmm.x<0.) discard;
    else {
        vec3 nor=(ray_origin+tmm.x*ray_dir)/2.;
        nor = reflect(ray_dir, nor);
        float fre = pow(.5+ clamp(dot(nor,ray_dir),0.0,1.0), 3. )*1.3;
        //col += texture(iChannel0, nor).rgb * fre;
        col += envcolor * fre;
    }

    // shade
    col =  .5 *(log(1.+col));
    col = clamp(col,0.,1.);
    fragColor = vec4( col, 1.0 );

    // trying to spherize the z-buffer
    // issue is that the sphere calculation uses a radius of 2
    // whereas the rendering uses a radius of 0.5
    // dividing tmm.x by 4 doesn't work
    // problem: if add sphere depth value, when the marble falls behind the
    // ground, it shows through, but without it, marble is blocked by low
    // geometry
    // perhaps test with quad the marble passes through along z axis
    //gl_FragDepth = gl_FragCoord.z; // - tmm.x;
}

void main() {
    mainImage(FragColor, uv);
}
