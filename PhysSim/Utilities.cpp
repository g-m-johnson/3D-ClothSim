#include "Utilities.h"

Play3d::Graphics::ComplexMaterialDesc CreateMaterialDescFromHLSL(const char* name, const char* hlslPath)
{
	char shaderName[64];

	Graphics::ShaderId customVertexShader;
	{
		sprintf_s(shaderName, 64, "%s_VS", name);
		Graphics::ShaderCompilerDesc compilerOptions = {};
		compilerOptions.m_name = shaderName;
		compilerOptions.m_type = Graphics::ShaderType::VERTEX_SHADER;
		compilerOptions.m_flags = Graphics::ShaderCompilationFlags::DEBUG | Graphics::ShaderCompilationFlags::SOURCE_FILE;
		compilerOptions.m_hlslCode = hlslPath;
		compilerOptions.m_entryPoint = "VS_Main";
		compilerOptions.m_defines.push_back({ "MAX_LIGHTS", "4" });
		customVertexShader = Graphics::Shader::Compile(compilerOptions);
		PLAY_ASSERT_MSG(customVertexShader.IsValid(), "Vertex Shader Compilation Failed!");
	}

	Graphics::ShaderId customPixelShader;
	{
		Graphics::ShaderCompilerDesc compilerOptions = {};
		compilerOptions.m_name = shaderName;
		compilerOptions.m_type = Graphics::ShaderType::PIXEL_SHADER;
		compilerOptions.m_flags = Graphics::ShaderCompilationFlags::DEBUG | Graphics::ShaderCompilationFlags::SOURCE_FILE;
		compilerOptions.m_hlslCode = hlslPath;
		compilerOptions.m_entryPoint = "PS_Main";
		compilerOptions.m_defines.push_back({ "MAX_LIGHTS", "4" });
		customPixelShader = Graphics::Shader::Compile(compilerOptions);
		PLAY_ASSERT_MSG(customPixelShader.IsValid(), "Pixel Shader Compilation Failed!");
	}

	Graphics::ComplexMaterialDesc desc;
	desc.m_state.m_cullMode = Graphics::CullMode::BACK;
	desc.m_state.m_fillMode = Graphics::FillMode::SOLID;
	desc.m_VertexShader = customVertexShader;
	desc.m_PixelShader = customPixelShader;

	return desc;
}
