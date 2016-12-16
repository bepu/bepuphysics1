float4x4 View;
float4x4 Projection;
#define NUM_TEXTURES 8
#define MAX_OBJECTS 81
float4x3 WorldTransforms[MAX_OBJECTS];
float TextureIndices[MAX_OBJECTS];

texture Colors;

float3 LightDirection1;
float3 DiffuseColor1;
float3 LightDirection2;
float3 DiffuseColor2;
float AmbientAmount;


sampler ColorSampler = sampler_state
{
    Texture = (Colors);
    
    MinFilter = Point;
    MagFilter = Point;
    MipFilter = Point;
    
    AddressU = Clamp;
    AddressV = Clamp;
};


//The texture coordinates aren't actually used in this shader but it makes things marginally simpler outside.  Not exactly optimized!
struct VertexShaderInput
{
    float4 Position : POSITION0;
    float3 Normal : NORMAL0;
    float2 TextureCoordinates : TEXCOORD0;
    float Index : TEXCOORD1;
	float TextureIndex : TEXCOORD2;
};

struct VertexShaderOutput
{
    float4 Position : POSITION0;
    float3 Normal : TEXCOORD0;
	float TextureIndex : TEXCOORD1;
};



VertexShaderOutput VertexShaderFunction(VertexShaderInput input)
{
    VertexShaderOutput output;
    int index = (int)round(input.Index);
    float3 worldPosition = mul(input.Position, WorldTransforms[index]);
    output.Position = mul(mul(float4(worldPosition, 1), View), Projection);
    output.Normal = mul(float4(input.Normal, 0), WorldTransforms[index]);
	output.TextureIndex = input.TextureIndex;

    return output;
}

float3 PixelShaderFunction(VertexShaderOutput input) : COLOR0
{

	float halfPixel = .5f / NUM_TEXTURES;
    
	float3 surfaceColor = tex2D(ColorSampler, float2(halfPixel + halfPixel * 2 * input.TextureIndex, halfPixel)).xyz;

		
	float3 normal = normalize(input.Normal);
    float diffuseAmount1 = saturate(-dot(normal, LightDirection1));
    float diffuseAmount2 = saturate(-dot(normal, LightDirection2));
  
    
	surfaceColor =  AmbientAmount * surfaceColor + surfaceColor * (diffuseAmount1 * DiffuseColor1 + diffuseAmount2 * DiffuseColor2);
    return surfaceColor;
}

technique Technique1
{
    pass Pass1
    {
        VertexShader = compile vs_4_0 VertexShaderFunction();
        PixelShader = compile ps_4_0 PixelShaderFunction();
    }
}
