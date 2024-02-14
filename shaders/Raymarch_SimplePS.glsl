#version 330

in vec3 normalPS;
in vec3 vertexPosWS;
in vec3 vertexPosOS;
out vec4 outColor;

uniform vec3 SHDCameraPos;
uniform vec3 SHDCameraDir;
uniform float SHD_Time;

uniform vec3 u_SphereCenter;
uniform mat3 u_RoundConeTransform;
uniform mat3 u_PrismTransform;
uniform mat3 u_CubeTransform;
uniform float u_CubeRoundess;
uniform mat3 u_TriangleTransform;
uniform vec3 u_LightPos;
uniform float u_Radius;
uniform float u_Blend;

// material uniforms
uniform vec3 u_BaseCol;
uniform float u_Smoothness;

#define MAXSTEPS 1000
#define SURFACE_DIST 0.001
#define MAX_DIST 250

mat4 RotationMatrix(vec3 axis, float angleDeg)
{
    //  float angle = radians(angleDeg);
    float angle = angleDeg;
    axis = normalize(axis);
    float s = sin(angle);
    float c = cos(angle);
    float oc = 1.0 - c;

    return mat4(oc * axis.x * axis.x + c, oc * axis.x * axis.y - axis.z * s, oc * axis.z * axis.x + axis.y * s, 0.0,
                oc * axis.x * axis.y + axis.z * s, oc * axis.y * axis.y + c, oc * axis.y * axis.z - axis.x * s, 0.0,
                oc * axis.z * axis.x - axis.y * s, oc * axis.y * axis.z + axis.x * s, oc * axis.z * axis.z + c, 0.0, 
                0.0, 0.0, 0.0, 1.0);
}

vec3 Rotate(vec3 v, vec3 axis, float angle)
{
    mat4 m = RotationMatrix(axis, angle);
    return (m * vec4(v, 1.0)).xyz;
}

vec3 TranformPoint(vec3 point, mat3 TransformMatrix)
{
    point = Rotate(point, vec3(1, 0, 0), TransformMatrix[1][0]);// rotation in X
    point = Rotate(point, vec3(0, 1, 0), TransformMatrix[1][1]);// rotation in Y
    point = Rotate(point, vec3(0, 0, 1), TransformMatrix[1][2]);// rotation in Z
    point += vec3(TransformMatrix[0][0], TransformMatrix[0][1], TransformMatrix[0][2]);// Translation
    return point;
}


float Fresnel(float cosT, float F0)
{
    return F0 + (1.0 - F0) * pow(1.0 - cosT, 5.0);
}

vec4 BlinnPhong(vec3 m_objectPos, vec3 m_objectNormal, vec3 m_lightPos, vec3 m_cameraPos, vec3 m_baseCol, float m_smoothness)
{
    vec3 lightDir = normalize(m_lightPos - m_objectNormal);
    vec3 viewDir = normalize(m_cameraPos - m_objectPos);
    vec3 reflectDir = reflect(-lightDir, m_objectNormal);

    float lambertDiffuse = max(0, dot(m_objectNormal, lightDir));
    float specular = max(0, dot(viewDir, reflectDir));
    specular = pow(specular, m_smoothness * 100) * m_smoothness; // adding specular control
    specular += mix(0, Fresnel(dot(m_objectNormal, viewDir), .05), m_smoothness); // adding fresnel

    vec4 col = vec4(lambertDiffuse * m_baseCol, 1);
    col += vec4(vec3(specular), 0);

    return col;
}

struct RaymarchModel
{
    vec3 Normal;
    vec3 Position;
    float Mask;
};

float Remap(float v, float inLow, float inHigh, float outLow, float outHigh) 
{
   return (((v - inLow) / (inHigh - inLow)) * (outHigh - outLow)) + outLow;
}

float RemapClamped(float v, float inLow, float inHigh, float outLow, float outHigh) 
{
   v = (((v - inLow) / (inHigh - inLow)) * (outHigh - outLow)) + outLow;
   return clamp(v, outLow, outHigh);
}

float smin(float a, float b, float k)
{
    float h = clamp(0.5 + 0.5 * (a - b) / k, 0.0, 1.0);
    return mix(a, b, h) - k * h * (1.0 - h);
}

float SphereSDF(vec3 point, float radius)
{
    return length(point) - radius;
}

float SphereRepeatSDF(vec3 point, float radius)
{
    // vec3 id = round(point/10);
    // vec3 r = point - 10*id;
    return length(point) - radius;
}

float CubeSDF(vec3 point)
{
    point = TranformPoint(point, u_CubeTransform);
    vec3 dimensions = vec3(u_CubeTransform[2][0], u_CubeTransform[2][1], u_CubeTransform[2][2]);
    vec3 cube = clamp(point, vec3(-dimensions.x, -dimensions.y, -dimensions.z), vec3(dimensions.x, dimensions.y, dimensions.z));
    return length(point - cube) - u_CubeRoundess;
}

float TriPrismSDF(vec3 point)
{
    // point = Rotate(point, vec3(1, 0, 0), u_PrismTransform[1][0]);// rotation in X
    // point = Rotate(point, vec3(0, 1, 0), u_PrismTransform[1][1]);// rotation in Y
    // point = Rotate(point, vec3(0, 0, 1), u_PrismTransform[1][2]);// rotation in Z
    // point += vec3(u_PrismTransform[0][0], u_PrismTransform[0][1], u_PrismTransform[0][2]);
    point = TranformPoint(point, u_PrismTransform);

    vec2 h = vec2(u_PrismTransform[2][0], u_PrismTransform[2][1]);
    vec3 q = abs(point);
    float prismOuter = max(q.z - h.y, max(q.x * 0.866025 + point.y * 0.5, -point.y) - h.x * 0.5);
    h.x -= u_PrismTransform[2][2];
    h.y += 0.1;
    float prismInner = max(q.z - h.y, max(q.x * 0.866025 + point.y * 0.5, -point.y) - h.x * 0.5);
    return max(prismOuter, -prismInner);
}

float CapsuleSDF(vec3 point)
{
    vec3 capsule = vec3(clamp(point.x, 0, 2), 0, 0); // clamp max is the height
    return length(point - capsule) - .2; // subtracting the radius
}

float RoundConeSDF(vec3 point)
{
    point = TranformPoint(point, u_RoundConeTransform);

    float r1 = u_RoundConeTransform[2][0];
    float r2 = u_RoundConeTransform[2][1];
    float h = u_RoundConeTransform[2][2];
  // sampling independent computations (only depend on shape)
    float b = (r1 - r2) / h;
    float a = sqrt(1.0 - b * b);

  // sampling dependant computations
    vec2 q = vec2(length(point.xz), point.y);
    float k = dot(q, vec2(-b, a));
    if(k < 0.0)
        return length(q) - r1;
    if(k > a * h)
        return length(q - vec2(0.0, h)) - r2;
    return dot(q, vec2(a, b)) - r1;
}

float EqTriangleSDF(vec3 p)
{
    p = TranformPoint(p, u_TriangleTransform);
    float height = u_TriangleTransform[2][0];
    float base = u_TriangleTransform[2][1];
    float r = u_TriangleTransform[2][2];
    vec3 a = vec3(0,1 + height*2,0);
    vec3 b = vec3(.75 + base,0,0);
    vec3 c = vec3(-.75 - base,0,0);

    vec3 pa = p - a, ba = b - a;
    float hAB = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
    float LineAB = length(pa - ba * hAB) - r;

    vec3 pb = p - b, cb = c - b;
    float hBC = clamp(dot(pb, cb) / dot(cb, cb), 0.0, 1.0);
    float LineBC = length(pb - cb * hBC) - r;

    vec3 pc = p - c, ac = a - c;
    float hCA = clamp(dot(pc, ac) / dot(ac, ac), 0.0, 1.0);
    float LineCA = length(pc - ac * hCA) - r;

    return min(min(LineAB, LineBC), LineCA);
}

void Animations()
{
    vec3 RoundCone = vec3(0, mix(.3, .01, Remap(SHD_Time, 0, .5, 0, 1)),0);
}

float WorldSDF(vec3 point)
{
    //Animations
    // float m_Time = round(SHD_Time/5);
    float m_Time = mod(SHD_Time, 5);
    vec3 a_RoundCone = vec3(0, mix(.3, -.01, RemapClamped(m_Time, 0, .5, 0, 1)),0);
    vec3 a_Triangle = vec3(0, (sin(SHD_Time * 5)+1)*.05, 0);
    
    float m_Cube = CubeSDF(point);
    float m_Sphere = SphereSDF(point + u_SphereCenter, u_Radius);
    
    float m_SphereRepeat = SphereRepeatSDF(point - 10*round(point/10), u_Radius);
    float m_Capsule = CapsuleSDF(point + u_SphereCenter);
    // float m_Arrow = ArrowSDF(point);
    float m_Prism = TriPrismSDF(point);
    float m_RoundCone = RoundConeSDF(point + a_RoundCone);
    float m_Triangle = EqTriangleSDF(point + a_Triangle);

    float outSDF = smin(m_Sphere, m_RoundCone, u_Blend);
    outSDF = min(outSDF, m_Triangle);
    return outSDF;
}

vec3 CalculateNormals(vec3 point)
{
   //  vec2 offset = vec2(0, .01);
   //  vec3 normal = normalize(point - vec3(WorldSDF(point - offset.yxx, u_Radius), WorldSDF(point - offset.xyx, u_Radius), WorldSDF(point - offset.xxy, u_Radius)));
   //  return normal;
    vec2 o = vec2(0, 0.01);
    vec3 normal;
    normal.r = WorldSDF(point + o.grr) - WorldSDF(point - o.grr);
    normal.g = WorldSDF(point + o.rgr) - WorldSDF(point - o.rgr);
    normal.b = WorldSDF(point + o.rrg) - WorldSDF(point - o.rrg);
    return normalize(normal);

}

RaymarchModel RayMarch(vec3 rayOrigin, vec3 rayDirection)
{
    RaymarchModel o;

    float distFromOrigin = 0; // Where the ray will orginate from. In this case the camera
    for(int i = 0; i < MAXSTEPS; ++i)
    {
        vec3 point = rayOrigin + (rayDirection * distFromOrigin);//The point the ray has marched to
        float distToScene = WorldSDF(point);
        distFromOrigin += distToScene; // moving forward in the scene

        o.Position = point;
        o.Normal = CalculateNormals(point);

        if(distToScene < SURFACE_DIST || distFromOrigin > MAX_DIST)
            break;
    }

    o.Mask = (distFromOrigin < MAX_DIST) ? 1 : 0;
    return o;
}

void main()
{
    //correcting uniforms
    float Smoothness = clamp(u_Smoothness, 0.05, 1);

    vec3 RayOrigin = SHDCameraPos;
    vec3 RayDirection = normalize(vertexPosWS - SHDCameraPos);

    RaymarchModel shape = RayMarch(RayOrigin, RayDirection);

   //lighting
    vec4 blinnPhongShade = BlinnPhong(shape.Position, shape.Normal, u_LightPos, SHDCameraPos, u_BaseCol, Smoothness);
    

    outColor.rgb = blinnPhongShade.rgb;
    outColor.a = shape.Mask;

    if(shape.Mask < 1)
        discard;
}