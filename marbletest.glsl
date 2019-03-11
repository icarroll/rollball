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

out vec4 FragColor;
in vec3 FragPos;

// ignored?
in vec3 Normal;
uniform vec3 lightPos;
uniform vec3 lightColor;
uniform vec3 objectColor;

vec3 iResolution = vec3(1.0, 1.0, 1.0);

vec3 envcolor = vec3(0.2, 0.3, 0.3);

uniform float iTime;

float zoom=1.;

vec2 cmul( vec2 a, vec2 b )  { return vec2( a.x*b.x - a.y*b.y, a.x*b.y + a.y*b.x ); }
vec2 csqr( vec2 a )  { return vec2( a.x*a.x - a.y*a.y, 2.*a.x*a.y  ); }

mat2 rot(float a) {
    return mat2(cos(a),sin(a),-sin(a),cos(a));  
}

vec2 iSphere( in vec3 ro, in vec3 rd, in vec4 sph )//from iq
{
    vec3 oc = ro - sph.xyz;
    float b = dot( oc, rd );
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

vec3 raymarch( in vec3 ro, vec3 rd, vec2 tminmax )
{
    float t = tminmax.x;
    //float dt = .1;
    float dt = .1 - .075*cos(iTime*.025);//animated
    vec3 col= vec3(0.);
    float c = 0.;
    for( int i=0; i<64; i++ )
    {
        t+=dt*exp(-2.*c);
        if(t>tminmax.y)break;
        
        //vec3 pos = refract( ro, (ro+t*rd)/2., 0.76); 
        //c = map(pos);
        
        vec3 pos = ro+t*rd;
        c = map(ro+t*rd);               
        
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
    //if( iMouse.z>0.0 )m = iMouse.xy/iResolution.xy*3.14;
    m-=.5;

    // camera
    vec3 ro = zoom*vec3(4.);
    ro.yz*=rot(m.y);
    ro.xz*=rot(m.x + 0.25*time);
    vec3 ta = vec3( 0.0 , 0.0, 0.0 );
    vec3 ww = normalize( ta - ro );
    vec3 uu = normalize( cross(ww,vec3(0.0,1.0,0.0) ) );
    vec3 vv = normalize( cross(uu,ww));
    vec3 rd = normalize( p.x*uu + p.y*vv + 4.0*ww )*0.975;

    
    vec2 tmm = iSphere( ro, rd, vec4(0.,0.,0.,2.) );

    // raymarch
    vec3 col = raymarch(ro,rd,tmm);
    //if (tmm.x<0.)col = texture(iChannel0, rd).rgb;
    if (tmm.x<0.)col = envcolor;
    else {
        vec3 nor=(ro+tmm.x*rd)/2.;
        nor = reflect(rd, nor);        
        float fre = pow(.5+ clamp(dot(nor,rd),0.0,1.0), 3. )*1.3;
        //col += texture(iChannel0, nor).rgb * fre;
        col += envcolor * fre;
    
    }
    
    // shade
    col =  .5 *(log(1.+col));
    col = clamp(col,0.,1.);
    fragColor = vec4( col, 1.0 );
}

void main() {
    mainImage(FragColor, FragPos.xy);
}
