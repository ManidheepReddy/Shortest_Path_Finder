#pragma once

#include <string>
#include <unordered_map>
#include <glm/glm.hpp>
class Shader
{
private:
	std::string mv_FilePath;
	std::string mf_FilePath;//for debug purpose
	unsigned int m_RendererID;
	std::unordered_map<std::string, int> m_UniformLocationCache; // Used for uniform caching
	//caching for uniforms
public:
	Shader(const std::string& v_filepath, const std::string& f_filepath);
	~Shader();

	void Bind() const;
	void Unbind() const;

	//Set uniforms
	void SetUniform1i(const std::string& name, int value);
	void SetUniform3f(const std::string& name, float v0, float v1, float v2);
	void SetUniform4f(const std::string& name, float v0, float v1, float f2, float f3);
	void SetUniformMat4f(const std::string& name, const glm::mat4& matrix);
private:
	std::string ParseShader(const std::string& file);
	unsigned int CompileShader(unsigned int type, const std::string& source);
	unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader);
	int GetUniformLocation(const std::string& name);
};