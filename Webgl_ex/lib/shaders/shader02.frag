// Fragment shader

precision highp float;
 
uniform vec2 iResolution;
uniform float iTime;
uniform int iType;
uniform int iSoftShadow;
uniform vec3 iRo;
uniform float iFov;
 
//---shadertoy code here--
 

#define MAX_STEPS 50
#define MAX_DIST 100.
#define SURF_DIST .01

/* Some unitility functions before we start*/
float acosh(float x){
    return log(x+sqrt(x*x - 1.));
}

/// COSH Function (Hyperbolic Cosine)
float cosh(float val)
{
    float tmp = exp(val);
    float cosH = (tmp + 1.0 / tmp) / 2.0;
    return cosH;
}
 
// TANH Function (Hyperbolic Tangent)
float tanh(float val)
{
    float tmp = exp(val);
    float tanH = (tmp - 1.0 / tmp) / (tmp + 1.0 / tmp);
    return tanH;
}
 
// SINH Function (Hyperbolic Sine)
float sinh(float val)
{
    float tmp = exp(val);
    float sinH = (tmp - 1.0 / tmp) / 2.0;
    return sinH;
}


/* Some operations in hyperbolic geometry*/



float HDot(vec4 v1, vec4 v2){
    /* H inner product*/
    return -v1.x *v2.x-v1.y *v2.y-v1.z *v2.z+v1.w *v2.w;
}


vec4 HNormalize(vec4 v){
    /* Normalize to +-1*/
    float norm = sqrt(abs(HDot(v,v)));
    return v/norm;
}

vec4 Euc2Hyp(vec3 p){
    /* This is NOT same as HNormalize
    Instead, This takes (x,y,z) and maps it to such a point (x,y,z,w) that it's on H3*/
    return vec4(p,sqrt(1.+p.x*p.x+p.y*p.y+p.z*p.z));
}


vec4 HMove(vec4 p, vec4 tangent, float t){
/*
Move along the geodesics at p in the tangent direction for distance t. 
Important: tangent is normalized. 
*/
    
    return p*cosh(t)+tangent*sinh(t);
}



vec4 FindTangent(vec4 p ,vec4 q){
/*
Takes two points p and q, and returns the tangent vector at p
*/
    float d = HDot(p,q);
    // CHECK THIS!!!!!!!!!!!!!!!!!!
    return (q-d*p)/sqrt(pow(d,2.)-1.);
    
}


float HLength(vec4 p,vec4 q){
/*
Compute distance between two points
*/
    return abs(acosh(HDot(p,q)));
}



/* Raymarching code*/

float sdSphere(vec4 pos, vec4 center, float rad,int type){
    float d;
    if (type == 1){
        d = length(pos.xyz-center.xyz)-rad;
    }
    else{
        d = HLength(pos, center)-rad;     
    }
    return d;
}

float sdHorosphere(vec4  pos, vec4 ipdirection, float offset, int type){
    if(type == 1){
        return log(dot(pos,ipdirection))-offset; 
    }else{
        return log(HDot(pos, ipdirection)) - offset;
    }

}


float GetDist(vec4 p, int type) {
    /* Get the minimum  distance of p with ANY object in the world*/
    // First object: sphere with center s
    vec4 s;
    float r = 1.;
    float d = 1e20;
    for (int i = 0;i<1;i++){
        for (int j = 0; j<1;j++){
            for (int k = 0; k<1;k++){
                s = Euc2Hyp (vec3(10.+3.*float(i),-10.+10.*float(j),-10.+10.*float(k)));
                d = min(d , sdSphere(p,s,r,type));
            }
        }

    }

    

    //vec4 s2 = Euc2Hyp (vec3(10.,0.,0.));
    //float r2 = 2.;
    //float sphereDist2 = sdSphere(p,s2,r2,type);
    
    //vec4 s3 = Euc2Hyp (vec3(10.,-10.,5.));
    //float r3 = 1.;
    //float sphereDist3 = sdHorosphere(p,s3,r3,type); 

    // Compute min distance
    return d;
} 




vec3 GetNormal(vec3 p ){
    /* Get the normal vector at point p*/ 
    
    float d = GetDist(vec4(p,1.),1);
    
    //Approximate the gradient of SDF at p
    vec2 e = vec2(.01,0);
    vec3 n = d-vec3(GetDist(vec4(p-e.xyy,1.),1),GetDist(vec4(p-e.yxy,1.),1), GetDist(vec4(p-e.yyx,1.),1));
    return normalize(n);
}

float RayMarch(vec3 ro, vec3 rd){
   	/*Take a ray origin and a normalized(important!) ray direction. 
	  Return the total ray marching distance from origin to the closest intersection point in the world.
    */ 
    
    float dO = 0.;
    
    // marching loop
    for(int i = 0; i<MAX_STEPS;i++){
        // move our current point along the ray (safely)
        vec3 p = ro+rd*dO;
        float dS = GetDist(vec4(p,1.),1);
        dO+= dS;
        if(dO>MAX_DIST || dS<SURF_DIST) break;
    }
    
    
    return dO;
}

float HRayMarch(vec4 ro, vec4 rd){
   	/*Take a ray origin and a normalized(important!) ray direction. 
	  Return the total ray marching distance from origin to the closest intersection point in the world.
    */ 
    
    float dO = 0.;
    
    // marching loop
    for(int i = 0; i<MAX_STEPS;i++){
        // move our current point along the ray (safely)
        vec4 p = HMove(ro,rd,dO);
        float dS = GetDist(p,2);
        dO+= dS;
        if(dO>MAX_DIST || dS<SURF_DIST*dO) break;
    }
    return dO;
}

void HGetTangentBasis(in vec4 p, out vec4 basisx, out vec4 basisy, out vec4 basisz){
    float d = GetDist(p,2);
    
    // Get three (independent) tangent vectors at p
    basisx = HNormalize(vec4(p.w,0.0,0.0,p.x));
    basisy = vec4(0.0,p.w,0.0,p.y);
    basisz = vec4(0.0,0.0,p.w,p.z);  
    
    // Use Gram Schmidt 
    basisy = HNormalize(basisy - HDot(basisy, basisx)*basisx); 
    basisz = HNormalize(basisz - HDot(basisz, basisx)*basisx - HDot(basisz, basisy)*basisy); 
}

vec4 HGetNormal(vec4 p ){
    /* Get the normal vector at point p*/ 
    vec4 basisx;
    vec4 basisy;
    vec4 basisz;
    HGetTangentBasis(p,basisx,basisy,basisz);
    
    float dx = GetDist(HMove(p,basisx,5.*SURF_DIST),2)-GetDist(HMove(p,basisx,-5.*SURF_DIST),2);
    float dy = GetDist(HMove(p,basisy,5.*SURF_DIST),2)-GetDist(HMove(p,basisy,-5.*SURF_DIST),2);
    float dz = GetDist(HMove(p,basisz,5.*SURF_DIST),2)-GetDist(HMove(p,basisz,-5.*SURF_DIST),2);
    //Approximate the gradient of SDF at p
    vec4 n = basisx *dx+basisy *dy+basisz *dz;
	
    
    return HNormalize(n);
}

float HShadowMarch(vec4 ro, vec4 rd, vec4 lightPos,float k, int is_soft){
   	/*Take a ray origin and a normalized(important!) ray direction. 
	  Return the total ray marching distance from origin to the closest intersection point in the world.
    */ 
    
    float light = 1.;
    if (is_soft == 2){
        return light;
    }
    float dO = SURF_DIST;
    float d = HLength(ro,lightPos);
    float previous_dS = 1e20;
    // marching loop
    for(int i = 0; i<MAX_STEPS;i++){
        // move our current point along the ray (safely)
        vec4 p = HMove(ro,rd,dO);
        float dS = GetDist(p,2);
        dO+= dS;
        if(is_soft == 1){
            light = min(light,k*dS/dO);         
        }

        if(dO>d) break;
        if (dS<SURF_DIST){
            light  = 0.;
            break;
        }
    }
    return light;
}


float ShadowMarch(vec3 ro, vec3 rd, vec3 lightPos,float k, int is_soft){
   	/*Take a ray origin and a normalized(important!) ray direction. 
	  Return the total ray marching distance from origin to the closest intersection point in the world.
    */ 
    
    float light = 1.;
    if (is_soft == 2){
        return light;
    }
    float dO = SURF_DIST;
    float d = length(lightPos-ro);
    float previous_dS = 1e20;
    // marching loop
    for(int i = 0; i<MAX_STEPS;i++){
        // move our current point along the ray (safely)
        vec3 p = ro+rd*dO;
        float dS = GetDist(vec4(p,1.),1);
        if (is_soft == 1){
            light = min(light,k*dS/dO);
        }
        dO+= dS;
        if(dO>d) break;
        if (dS<SURF_DIST){
            light  = 0.;
            break;
        }
    }
    return clamp(light,0.,1.);
}




float GetLight(vec4 p, vec4 ro, int type){
    /* Get LightValue at point p Using Phong Model*/ 
    // Light Position
    vec4 lightPos = Euc2Hyp(vec3(10,20.*cos(iTime),20.*sin(iTime)));


    float lightIntensity = 1.;
    float light;
    if (type == 1){
            // The light vector with respect to p
        vec3 l = normalize(lightPos.xyz-p.xyz);
    
        // The normal vector at p
        vec3 n = GetNormal(p.xyz);
    
        // The view vector with respect to p
        vec3 v = normalize(ro.xyz-p.xyz);
    
        // The mid-vector between l and v
        vec3 h = normalize(v+l);
  
        // kd = diffuse coefficient, ks  = specular coefficient 
        float kd = 1.;
        float ks = .2;
        float dif = kd*lightIntensity*max(dot(n,l),0.);
        float spec = ks*lightIntensity*max(dot(n,h),0.);
    
        // add diffuse and specular
        light = dif+spec;
    
        //Cast Shadow by shooting another ray marching to light source. 
        float s = ShadowMarch(p.xyz+2.*n*SURF_DIST,l,lightPos.xyz,4.,iSoftShadow);
        light  = light*s;
    
    }else{
        vec4 l = FindTangent(p,lightPos);
    
        // The normal vector at p
        vec4 n = HGetNormal(p);
    
        // The view vector with respect to p
        vec4 v = FindTangent(p,ro);
    
        // The mid-vector between l and v
        vec4 h = HNormalize(v+l);
  
        // kd = diffuse coefficient, ks  = specular coefficient 
        float kd = 1.;
        float ks = .2;
        float dif = -kd*lightIntensity*HDot(n,l);
        float spec = -ks*lightIntensity*HDot(n,h);
    
        // add diffuse and specular
        float light = dif+spec;
    
        //Cast Shadow by shooting another marching to light source. 
        float d = HShadowMarch(HMove(p,n,40.*SURF_DIST),l,lightPos,4.,iSoftShadow);
        return light*d;    
    }

    
    return light;
}




float render(vec2 uv, int type, vec4 ro, float fov){
    float light;
    if(type == 1){
        vec3 rd = normalize(vec3(fov,uv.x,uv.y));
        float d = RayMarch(ro.xyz,rd);
        vec3 p = ro.xyz+rd*d;
        // Get the lighting info at that point
        light = GetLight(vec4(p,1.),ro,1);
    }
    else{
        vec4 rd;    
        if(type == 2){
            vec4 tangentu;
            vec4 tangentv;
            vec4 tangento;
            HGetTangentBasis(ro,tangento,tangentu,tangentv);
            // Get the closest intersection point from ro at rd direction
            rd = HNormalize(fov*tangento+uv.y*tangentv+uv.x*tangentu);
        }else if(type == 3){
            vec4 screenpoint = HNormalize(vec4(fov,uv,1.));
            rd = FindTangent(ro,screenpoint);            
        }else if (type == 4){
            float prod = pow(uv.x,2.)+pow(uv.y,2.)+pow(fov,2.);
            vec4 screenpoint =  vec4(2.*fov,2.*uv,1.+prod)/(1.-prod);
            rd = FindTangent(ro,screenpoint);
        }
        else{
            return 1.;
        }
        float d = HRayMarch(ro,rd);
        vec4 p = HMove(ro,rd,d);
        light = GetLight(p,ro,2);    
    }
    // Get the lighting info at that point
    
    return light;
    
}



void mainImage( out vec4 fragColor, in vec2 fragCoord )
{
    // type 1:Euclidean 
    // type 2:Hyperbolic Tangent Space projection
    // type 3:Hyperbolic Klein Projection
    // type 4:Hyperbolic Poincere Disk Projection 
    
    
    // adjusted uv 
    vec2 uv = (fragCoord-.5*iResolution.xy)/max(iResolution.x,iResolution.y);
    //issue: There are points that are not on the unit disk/sphere, which cannot be projected to the hyperboloid.

    float light = render(uv,iType, Euc2Hyp(iRo),iFov);
    
    fragColor = vec4(light,0.,0.,1.);   
}





 
void main() {
  mainImage(gl_FragColor, gl_FragCoord.xy);
}
