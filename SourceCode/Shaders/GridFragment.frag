#version 330 core
in vec3 vs_position;
in vec3 vs_color;
in vec2 vs_texcoord;
in vec3 vs_normal;

out vec4 fs_color;
uniform sampler2D texture0;            //Color 
uniform int anchor_mark;

 void main()
{
	if (anchor_mark == -1)
		fs_color = vec4(vs_color, 1);
	else if (anchor_mark == 0)
		fs_color = vec4(1.,0.,0.,1.);
	else
		fs_color = vec4(0.,1.,0.,1.);
}