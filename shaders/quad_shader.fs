#version 330 core

in vec2 fragmentTexCoords;
out vec4 finalColor;

uniform sampler2D screenTexture;
uniform bool hdr;

void main()
{
	//finalColor = vec4(1.0, 0.0, 0.0, 1.0);
	if (hdr) {
		const float gamma = 2.2;

		vec3 hdrColor = texture(screenTexture, fragmentTexCoords).rgb;

		// *** REINHARD TONE MAPPING ***
		vec3 mapped = hdrColor / (hdrColor + vec3(1.0, 1.0, 1.0));

		// gamma
		mapped = pow(mapped, vec3(1.0 / gamma));

		finalColor = vec4(mapped, 1.0);
	} else {
		finalColor = texture(screenTexture, fragmentTexCoords);
	}
}