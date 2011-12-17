float4x4 View;
float4x4 Projection;
#define NUM_TEXTURES 8
#define MAX_OBJECTS 61
float4x4 WorldTransforms[MAX_OBJECTS];
float TextureIndices[MAX_OBJECTS];

texture Texture0;
texture Texture1;
texture Texture2;
texture Texture3;
texture Texture4;
texture Texture5;
texture Texture6;
texture Texture7;

float3 LightDirection1;
float3 DiffuseColor1;
float3 LightDirection2;
float3 DiffuseColor2;
float AmbientAmount;


sampler TextureSamplers[NUM_TEXTURES] = 
{

sampler_state
{
    Texture = (Texture0);
    
    MinFilter = Linear;
    MagFilter = Linear;
    MipFilter = Linear;
    
    AddressU = Wrap;
    AddressV = Wrap;
},

sampler_state
{
    Texture = (Texture1);
    
    MinFilter = Linear;
    MagFilter = Linear;
    MipFilter = Linear;
    
    AddressU = Wrap;
    AddressV = Wrap;
},

sampler_state
{
    Texture = (Texture2);
    
    MinFilter = Linear;
    MagFilter = Linear;
    MipFilter = Linear;
    
    AddressU = Wrap;
    AddressV = Wrap;
},

sampler_state
{
    Texture = (Texture3);
    
    MinFilter = Linear;
    MagFilter = Linear;
    MipFilter = Linear;
    
    AddressU = Wrap;
    AddressV = Wrap;
},

sampler_state
{
    Texture = (Texture4);
    
    MinFilter = Linear;
    MagFilter = Linear;
    MipFilter = Linear;
    
    AddressU = Wrap;
    AddressV = Wrap;
},

sampler_state
{
    Texture = (Texture5);
    
    MinFilter = Linear;
    MagFilter = Linear;
    MipFilter = Linear;
    
    AddressU = Wrap;
    AddressV = Wrap;
},

sampler_state
{
    Texture = (Texture6);
    
    MinFilter = Linear;
    MagFilter = Linear;
    MipFilter = Linear;
    
    AddressU = Wrap;
    AddressV = Wrap;
},

sampler_state
{
    Texture = (Texture7);
    
    MinFilter = Linear;
    MagFilter = Linear;
    MipFilter = Linear;
    
    AddressU = Wrap;
    AddressV = Wrap;
}

};


struct VertexShaderInput
{
    float4 Position : POSITION0;
    float3 Normal : NORMAL0;
    float2 TextureCoordinates : TEXCOORD0;
    float Index : TEXCOORD1;
};

struct VertexShaderOutput
{
    float4 Position : POSITION0;
    float3 Normal : NORMAL0;
    float2 TextureCoordinates : TEXCOORD0;
    float Index : TEXCOORD1;
};



VertexShaderOutput VertexShaderFunction(VertexShaderInput input)
{
    VertexShaderOutput output;
    int index = (int)round(input.Index);
    float4 worldPosition = mul(input.Position, WorldTransforms[index]);
    output.Position = mul(mul(worldPosition, View), Projection);
    output.TextureCoordinates = input.TextureCoordinates;
    output.Normal = mul(input.Normal, WorldTransforms[index]);
    output.Index = input.Index;	

    return output;
}

float4 PixelShaderFunction(VertexShaderOutput input) : COLOR0
{
    float index = round(input.Index);
    int x = TextureIndices[index];
    float4 surfaceColor;
    
    if(x == 7)
		surfaceColor = tex2D(TextureSamplers[0], input.TextureCoordinates);
	else if(x == 6)
		surfaceColor = tex2D(TextureSamplers[1], input.TextureCoordinates);
	else if(x == 5)
		surfaceColor = tex2D(TextureSamplers[2], input.TextureCoordinates);
	else if(x == 4)
		surfaceColor = tex2D(TextureSamplers[3], input.TextureCoordinates);
	else if(x == 3)
		surfaceColor = tex2D(TextureSamplers[4], input.TextureCoordinates);
	else if(x == 2)
		surfaceColor = tex2D(TextureSamplers[5], input.TextureCoordinates);
	else if(x == 1)
		surfaceColor = tex2D(TextureSamplers[6], input.TextureCoordinates);
	else
		surfaceColor = tex2D(TextureSamplers[7], input.TextureCoordinates);
		
	float3 normal = normalize(input.Normal);
    float diffuseAmount1 = saturate(-dot(normal, LightDirection1));
    float diffuseAmount2 = saturate(-dot(normal, LightDirection2));
  
    
	surfaceColor =  float4(AmbientAmount * surfaceColor + surfaceColor * (diffuseAmount1 * DiffuseColor1 + diffuseAmount2 * DiffuseColor2), 1);
    return surfaceColor;
}

technique Technique1
{
    pass Pass1
    {
        VertexShader = compile vs_3_0 VertexShaderFunction();
        PixelShader = compile ps_3_0 PixelShaderFunction();
    }
}
