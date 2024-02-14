#version 330

in vec3 normalPS;
in vec3 vertexPosWS;
in vec3 vertexPosOS;
out vec4 outColor;

uniform vec3 SHDCameraPos;
uniform vec3 SHDCameraDir;

uniform vec3 u_SphereCenter;
uniform vec3 u_LightPos;
uniform float u_Radius;
uniform float u_Blend;
uniform float u_Smoothness;

#define MAXSTEPS 100
#define SURFACE_DIST 0.001
#define MAX_DIST 1000

struct SDFModel
{
    vec3 Normal;
    vec3 Position;
    float Mask;
};

float smin(float a, float b, float k)
{
    float h = clamp(0.5 + 0.5 * (a - b) / k, 0.0, 1.0);
    return mix(a, b, h) - k * h * (1.0 - h);
}

float Fresnel(float cosT, float F0)
{
    return F0 + (1.0 - F0) * pow(1.0 - cosT, 5.0);
}

float SphereSDF(vec3 point, float radius)
{
    return length(point) - radius;
}

float CubeSDF(vec3 point, float radius)
{
    vec3 cube = clamp(point, vec3(-radius), vec3(radius));
    return length(point - cube);
}

vec4 WorldSDF(vec3 point)
{
    float m_Cube = CubeSDF(point, u_Radius);
    // return SphereSDF(point, radius);
    return smin(CubeSDF(point, u_Radius), SphereSDF(point + u_SphereCenter, u_Radius), u_Blend);
}

vec3 CalculateNormals(vec3 point)
{
    vec2 offset = vec2(0, .01);
    vec3 normal = normalize(point - vec3(WorldSDF(point - offset.yxx, u_Radius), WorldSDF(point - offset.xyx, u_Radius), WorldSDF(point - offset.xxy, u_Radius)));
    return normal;
}

SDFModel RayMarch(vec3 rayOrigin, vec3 rayDirection)
{
    SDFModel o;

    float distFromOrigin = 0; // Where the ray will orginate from. In this case the camera
    for(int i = 0; i < MAXSTEPS; ++i)
    {
        vec3 point = rayOrigin + (rayDirection * distFromOrigin);//The point the ray has marched to
        vec4 SDFShape = WorldSDF(point);
        float distToScene = SDFShape.a;
        distFromOrigin += distToScene; // moving forward in the scene

        o.Position = point;
        o.Normal = SDFModel.rgb;

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

    SDFModel shape = RayMarch(RayOrigin, RayDirection);

   //lighting
    vec3 lightDir = normalize(u_LightPos - shape.Normal);
    vec3 viewDir = normalize(SHDCameraPos - shape.Position);
    vec3 reflectDir = reflect(-lightDir, shape.Normal);

    float lambertDiffuse = max(0, dot(shape.Normal, lightDir));
    float specular = max(0, dot(viewDir, reflectDir));
    specular = pow(specular, Smoothness * 100) * Smoothness;

    float phong = lambertDiffuse + specular;
    phong += (Fresnel(dot(shape.Normal, viewDir), .05) * Smoothness);

    outColor.rgb = vec3(phong);
    outColor.b += 0.2;

    outColor.a = shape.Mask;

    if(shape.Mask < 1)
        discard;
}