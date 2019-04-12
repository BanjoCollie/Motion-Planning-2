// Includes and defs --------------------------

// openGL functionality
#include <glad/glad.h>
#include <glad/glad.c>
#include <GLFW/glfw3.h>
// shader helper
#include <learn_opengl/shader.h>
// math
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <random>
#include <time.h>
#include <vector>

#include "model.h"

// image loading
#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>


// Functions ---------------------------------

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void create_roadmap();
void addAgent(glm::vec3 start, glm::vec3 goal);
bool collidesWithObs(glm::vec2 point1, glm::vec2 point2);
bool lineIntersection(glm::vec2 p1, glm::vec2 p2, glm::vec2 q1, glm::vec2 q2);

// Global variables ---------------------------

// window
const int SCR_WIDTH = 1280;
const int SCR_HEIGHT = 720;

// camera
glm::vec3 cameraPos = glm::vec3(0.0f, 3.0f, 0.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
float yaw = -90.0f, pitch = 0.0f;
bool firstMouse = true;
float lastX = SCR_WIDTH / 2.0f, lastY = SCR_HEIGHT / 2.0f;

// time
float deltaTime = 0.0f;	// Time between current frame and last frame
float lastFrame = 0.0f; // Time of last frame

// General
float mapSize = 40.0f;
const int numNewPos = 150;
const int numAgents = 16;
std::vector<glm::vec3> points;
std::vector<unsigned int> edges[numNewPos + (numAgents*2)]; // For searching
std::vector<unsigned int> edgeIndices; // For drawing roadmap
std::vector<unsigned int> paths[numAgents];
bool aStar = false;


// Agents
bool moveAgents = false;
bool showPoints = false;
bool showEdges = false;
std::vector<glm::vec3> agentPos;
std::vector<glm::vec3> agentVel;
std::vector<glm::vec3> forceAccum;
float agentRad = 0.49f;
std::vector<glm::vec3> agentGoals;
std::vector<glm::vec3> nextPathPoint;
std::vector<int> startIndices;
std::vector<int> goalIndices;

// Barrels
std::vector<glm::vec3> barrelPos;
float barrelRad = 1.0f;
float barrelRadCoord;

// Cars
std::vector<glm::vec3> carPos;
float carX = 2.5;
float carZ = 1.25;
std::vector<bool> carRot;

int main()
{

	// Before loop starts ---------------------
	// glfw init
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// glfw window creation
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Motion Planning", NULL, NULL);
	glfwMakeContextCurrent(window);

	// register callbacks
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetKeyCallback(window, key_callback);
	glfwSetCursorPosCallback(window, mouse_callback);

	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// Initialize glad
	gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

	// Enable openGL settings
	//glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);

	// Setup ----------------------------------

	// AI variables
	barrelRadCoord = barrelRad + agentRad;
	carX += agentRad;
	carZ += agentRad;

	/*
	addAgent(glm::vec3(15.0f, 0.0f, 10.0f), glm::vec3(-15.0f, 0.0f, -10.0f));
	addAgent(glm::vec3(0.0f, 0.0f, 10.0f), glm::vec3(0.0f, 0.0f, -10.0f));
	addAgent(glm::vec3(10.0f, 0.0f, -5.0f), glm::vec3(-10.0f, 0.0f, 5.0f));
	addAgent(glm::vec3(-15.0f, 0.0f, 5.0f), glm::vec3(19.0f, 0.0f, 13.0f));
	addAgent(glm::vec3(13.0f, 0.0f, 5.0f), glm::vec3(-7.0f, 0.0f, -11.0f));
	addAgent(glm::vec3(-19.0f, 0.0f, -1.0f), glm::vec3(13.0f, 0.0f, 8.0f));
	addAgent(glm::vec3(-15.0f, 0.0f, -10.0f), glm::vec3(15.0f, 0.0f, 10.0f));
	addAgent(glm::vec3(0.0f, 0.0f, -10.0f), glm::vec3(0.0f, 0.0f, 10.0f));
	addAgent(glm::vec3(-10.0f, 0.0f, 5.0f), glm::vec3(10.0f, 0.0f, -5.0f));
	addAgent(glm::vec3(10.0f, 0.0f, 5.0f), glm::vec3(-9.0f, 0.0f, -13.0f));
	addAgent(glm::vec3(-3.0f, 0.0f, -2.0f), glm::vec3(3.0f, 0.0f, -12.0f));
	addAgent(glm::vec3(19.0f, 0.0f, 1.0f), glm::vec3(-13.0f, 0.0f, -8.0f));
	*/
	addAgent(glm::vec3(15.0f, 0.0f, 10.0f), glm::vec3(-15.0f, 0.0f, 10.0f));
	addAgent(glm::vec3(15.0f, 0.0f, 9.0f), glm::vec3(-15.0f, 0.0f, 9.0f));
	addAgent(glm::vec3(15.0f, 0.0f, 8.0f), glm::vec3(-15.0f, 0.0f, 8.0f));
	addAgent(glm::vec3(15.0f, 0.0f, 7.0f), glm::vec3(-15.0f, 0.0f, 7.0f));
	addAgent(glm::vec3(-15.0f, 0.0f, 10.2f), glm::vec3(15.0f, 0.0f, 10.2f));
	addAgent(glm::vec3(-15.0f, 0.0f, 9.2f), glm::vec3(15.0f, 0.0f, 9.2f));
	addAgent(glm::vec3(-15.0f, 0.0f, 8.2f), glm::vec3(15.0f, 0.0f, 8.2f));
	addAgent(glm::vec3(-15.0f, 0.0f, 7.2f), glm::vec3(15.0f, 0.0f, 7.2f));

	addAgent(glm::vec3(16.0f, 0.0f, 10.0f), glm::vec3(-16.0f, 0.0f, 10.0f));
	addAgent(glm::vec3(16.0f, 0.0f, 9.0f), glm::vec3(-16.0f, 0.0f, 9.0f));
	addAgent(glm::vec3(16.0f, 0.0f, 8.0f), glm::vec3(-16.0f, 0.0f, 8.0f));
	addAgent(glm::vec3(16.0f, 0.0f, 7.0f), glm::vec3(-16.0f, 0.0f, 7.0f));
	addAgent(glm::vec3(-16.0f, 0.0f, 10.2f), glm::vec3(16.0f, 0.0f, 10.2f));
	addAgent(glm::vec3(-16.0f, 0.0f, 9.2f), glm::vec3(16.0f, 0.0f, 9.2f));
	addAgent(glm::vec3(-16.0f, 0.0f, 8.2f), glm::vec3(16.0f, 0.0f, 8.2f));
	addAgent(glm::vec3(-16.0f, 0.0f, 7.2f), glm::vec3(16.0f, 0.0f, 7.2f));

	barrelPos.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
	barrelPos.push_back(glm::vec3(3.0f, 0.0f, 1.0f));
	barrelPos.push_back(glm::vec3(-12.0f, 0.0f, 2.0f));
	barrelPos.push_back(glm::vec3(19.0f, 0.0f, -4.0f));
	barrelPos.push_back(glm::vec3(0.0f, 0.0f, -16.0f));
	barrelPos.push_back(glm::vec3(8.0f, 0.0f, 2.0f));

	//*
	carPos.push_back(glm::vec3(10.0f, 0.0f, 0.0f));
	carRot.push_back(false);
	carPos.push_back(glm::vec3(-15.0f, 0.0f, -6.0f));
	carRot.push_back(false);
	carPos.push_back(glm::vec3(7.0f, 0.0f, 16.0f));
	carRot.push_back(false);
	//*/

	//*
	// Floor
	float floorVertices[] = {
		//x			y		z			nX		nY		nZ		t		s
		-1.0f,		0.0f,	-1.0f,		0.0f,	1.0f,	0.0f,	0.0f, 0.0f,
		-1.0f,		0.0f,	1.0f,		0.0f,	1.0f,	0.0f,	0.0f, 8.0f,
		 1.0f,		0.0f,	-1.0f,		0.0f,	1.0f,	0.0f,	8.0f, 0.0f,
		 1.0f,		0.0f,	1.0f,		0.0f,	1.0f,	0.0f,	8.0f, 8.0f
	};
	int floorIndices[] = {
		0, 1, 2,
		1, 3, 2
	};
	// Buffer stuff for floor
	unsigned int floorVAO, floorVBO, floorEBO;
	glGenVertexArrays(1, &floorVAO);
	glBindVertexArray(floorVAO);
	glGenBuffers(1, &floorVBO);
	glBindBuffer(GL_ARRAY_BUFFER, floorVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(floorVertices), floorVertices, GL_STATIC_DRAW);
	glGenBuffers(1, &floorEBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, floorEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(floorIndices), floorIndices, GL_STATIC_DRAW);
	// Tell OpenGL how to use vertex data
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0); //Uses whatever VBO is bound to GL_ARRAY_BUFFER
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);

	
	// Floor texture
	// Set up textures
	unsigned int dirtTexture;
	glGenTextures(1, &dirtTexture);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, dirtTexture);
	// load and generate the texture
	int width, height, nrChannels; 
	//stbi_set_flip_vertically_on_load(true);
	unsigned char *data = stbi_load("Dirt_01.jpg", &width, &height, &nrChannels, 0);
	if (data)
	{
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
		glGenerateMipmap(GL_TEXTURE_2D);
	}
	else
	{
		std::cout << "Failed to load texture" << std::endl;
	}
	stbi_image_free(data);
	//*/
	//Shader
	Shader texturedShader("textured.vert", "textured.frag");

	// Car
	Model car("car/new_jeep_dl.obj");
	Model barrel("barrel/barrel.obj");
	Model robot("robot/brain-robot.obj");


	// uncomment this call to draw in wireframe polygons.
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	//create_roadmap();

	unsigned int pointVAO, pointVBO, edgeEBO;
	glGenVertexArrays(1, &pointVAO);
	glBindVertexArray(pointVAO);

	glGenBuffers(1, &pointVBO);
	glBindBuffer(GL_ARRAY_BUFFER, pointVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3)*points.size(), points.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, (void*)0);
	glEnableVertexAttribArray(0);

	glGenBuffers(1, &edgeEBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edgeEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * edgeIndices.size(), edgeIndices.data(), GL_STATIC_DRAW);

	// render loop ----------------------------
	while (!glfwWindowShouldClose(window))
	{
		// Set deltaT
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		// input
		processInput(window);

		// processing

		// Move agents

		if (moveAgents)
		{
			for (int i = 0; i < agentPos.size(); i++)
			{
				forceAccum[i] = glm::vec3(0.0f);
				//agentVel[i] = glm::vec3(0.0f);
			}

			for (int agent = 0; agent < agentPos.size(); agent++)
			{
				// First check if you can see the next point, if you can move towards that instead
				float agentSpeed = 2.0f;
				if (nextPathPoint[agent] != agentGoals[agent])
				{
					glm::vec3 nextPoint = points[paths[agent].back()];
					glm::vec2 p1 = glm::vec2(agentPos[agent][0], agentPos[agent][2]);
					glm::vec2 p2 = glm::vec2(nextPoint[0], nextPoint[2]);
					if (!collidesWithObs(p1, p2))
					{
						// Can see next point
						nextPathPoint[agent] = nextPoint;
						paths[agent].pop_back();
						//std::cout << "reached point" << std::endl;
					}
				}

				// Now get a goal force

				//glm::vec3 vel = glm::normalize(nextPathPoint[agent] - agentPos[agent]) * agentSpeed * deltaTime;
				//if (vel[0] == vel[0])
				//	agentPos[agent] += vel;
				glm::vec3 offset = (nextPathPoint[agent] - agentPos[agent]);
				glm::vec3 goalVel;
				if (offset[0] == offset[0]) //If offset is defined
				{
					if (glm::length(offset) > 1.0f)
					{
						offset = glm::normalize(offset);
						goalVel = 3.0f * offset;
					}
					else
						goalVel = offset;
				}
				else
				{
					goalVel = glm::vec3(0.0f);
				}

				glm::vec3 goalForce = 2.0f * (goalVel - agentVel[agent]);

				forceAccum[agent] = goalForce;

				// Now need TTC force from other agents
				for (int otherA = 0; otherA < agentPos.size(); otherA++)
				{
					// For each other agent
					if (otherA != agent)
					{
						// First get tau
						float tau;

						float r = agentRad * 2.0f;
						glm::vec3 w = agentPos[agent] - agentPos[otherA];
						float c = glm::dot(w, w) - r * r;
						if (c < 0)
						{
							tau = 0.0f;
						}
						else
						{
							glm::vec3 v = -agentVel[agent] + agentVel[otherA]; //Reversed for some reason?
							float a = glm::dot(v, v);
							float b = glm::dot(w, v);
							float discr = b * b - a * c;
							if (discr <= 0.0f)
							{
								tau = INFINITY;
							}
							else
							{
								tau = (b - sqrt(discr)) / a;
								if (tau < 0) { tau = INFINITY; }
							}
						}
					// Got tau

					glm::vec3 dir = (agentPos[agent] + agentVel[agent] * tau) - (agentPos[otherA] + agentVel[otherA] * tau);
					if (dir[0] != 0.0f)
						dir = glm::normalize(dir);

					float horizon = 20.0f;
					float mag = 0.0f;
					if (tau >= 0 && tau <= horizon)
					{
						mag = (horizon - tau) / (tau + 0.001f);
					}
					if (mag > 10.0f)
						mag = 10.0f;
					
					if ((mag*dir)[0] == (mag*dir)[0])
						forceAccum[agent] += mag*dir;
					}
				}

			}
			

			//Integrate forces
			for (int agent = 0; agent < agentPos.size(); agent++)
			{
				agentVel[agent] += forceAccum[agent] * deltaTime;
				agentPos[agent] += agentVel[agent] * deltaTime;
			}
			//moveAgents = false;
		}


		// rendering commands here
		glClearColor(0.2f, 0.4f, 0.4f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
		glm::mat4 projection = glm::mat4(1.0f);
		projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
		glm::mat4 model = glm::mat4(1.0f);

		//*
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, dirtTexture);
		texturedShader.use();
		texturedShader.setMat4("view", view);
		texturedShader.setMat4("projection", projection);

		texturedShader.setVec3("viewPos", cameraPos);
		texturedShader.setVec3("light.direction", glm::vec3(0.0f, -1.0f, 1.0f));
		texturedShader.setVec3("light.ambient", glm::vec3(0.3f, 0.3f, 0.3f));
		texturedShader.setVec3("light.diffuse", glm::vec3(0.9f, 0.9f, 0.9f));
		texturedShader.setVec3("light.specular", glm::vec3(1.0f, 1.0f, 1.0f));

		texturedShader.setInt("material.diffuse", 0);
		texturedShader.setVec3("material.specular", glm::vec3(0.5f, 0.5f, 0.5f));
		texturedShader.setFloat("material.shininess", 0.1f);
		glBindVertexArray(floorVAO);
		
		model = glm::scale(model, glm::vec3(mapSize/2.0f, 1.0f, mapSize/2.0f));
		texturedShader.setMat4("model", model);
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
		model = glm::mat4(1.0f);
		//*/

		// Truck is 5m x ?m x 2.5m by default, 2.1m above ground
		//*
		for (int i = 0; i < carPos.size(); i++)
		{
			model = glm::translate(model, glm::vec3(0.0f, 1.05f, 0.0f));
			model = glm::translate(model, carPos[i]);
			model = glm::scale(model, glm::vec3(0.5f)); //~2.5 x 1.25 now
			if (carRot[i])
			{
				model = glm::rotate(model, 3.14159265f / 2.0f, glm::vec3(0.0f, 1.0f, 0.0f));
			}
			texturedShader.setMat4("model", model);
			car.Draw(texturedShader);
			model = glm::mat4(1.0f);
		}
		//*/

		// Barrel is 0.31m radius circle by default, on ground level
		for (int i = 0; i < barrelPos.size(); i++)
		{
			model = glm::translate(model, barrelPos[i]);
			model = glm::scale(model, glm::vec3(2.0f)); //~1m radius now
			texturedShader.setMat4("model", model);
			barrel.Draw(texturedShader);
			model = glm::mat4(1.0f);
		}

		// Robot it 2m radius circle by default, 3m above ground
		for (int i = 0; i < agentPos.size(); i++)
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.75f, 0.0f));
			model = glm::translate(model, agentPos[i]);
			model = glm::scale(model, glm::vec3(0.25f)); //~0.5m radius now
			texturedShader.setMat4("model", model);
			robot.Draw(texturedShader);
			model = glm::mat4(1.0f);
		}

		if (showPoints)
		{
			// Points
			model = glm::translate(model, glm::vec3(0.0f, 1.0f, 0.0f));
			texturedShader.setMat4("model", model);
			glBindVertexArray(pointVAO);
			glPointSize(10.0f);
			glDrawArrays(GL_POINTS, 0, points.size());

			if (showEdges)
				glDrawElements(GL_LINES, edgeIndices.size(), GL_UNSIGNED_INT, 0);
		}

		// check and call events and swap the buffers
		glfwPollEvents();
		glfwSwapBuffers(window);
	}

	glfwTerminate();

	//while (true) {} // Uncomment to see output after you close window

	return 0;
}

// This function is called whenever window is resized
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}

// Process all ketboard input here
void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	float cameraSpeed = 4.0f * deltaTime;
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		cameraPos += cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		cameraPos -= cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
		cameraPos += cameraSpeed * cameraUp;
	if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
		cameraPos -= cameraSpeed * cameraUp;

}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
		moveAgents = !moveAgents;

	if (key == GLFW_KEY_F && action == GLFW_PRESS)
	{
		float startTime = glfwGetTime();
		cout << "Building roadmap and running A*" << endl;
		aStar = true;
		create_roadmap();
		float endTime = glfwGetTime();
		cout << "Elapsed time was: " << endTime - startTime << endl;
	}
	if (key == GLFW_KEY_G && action == GLFW_PRESS)
	{
		float startTime = glfwGetTime();
		cout << "Building roadmap and running uniform cost search" << endl;
		aStar = false;
		create_roadmap();
		float endTime = glfwGetTime();
		cout << "Elapsed time was: " << endTime - startTime << endl;
	}

	if (key == GLFW_KEY_0 && action == GLFW_PRESS)
	{
		showPoints = !showPoints;
		showEdges = !showEdges;
	}

	if (key == GLFW_KEY_1 && action == GLFW_PRESS)
	{
		// Place barrel
		// cameraPos + cameraFront * d = 0 (looking at only y)
		// d = -cameraPos/cameraFront
		float dist = -cameraPos[1] / cameraFront[1];
		float x = cameraPos[0] + cameraFront[0] * dist;
		float z = cameraPos[2] + cameraFront[2] * dist;

		if (dist > 0)
			barrelPos.push_back(glm::vec3(x, 0.0f, z));
	}
	if (key == GLFW_KEY_2 && action == GLFW_PRESS)
	{
		// Place car
		// cameraPos + cameraFront * d = 0 (looking at only y)
		// d = -cameraPos/cameraFront
		float dist = -cameraPos[1] / cameraFront[1];
		float x = cameraPos[0] + cameraFront[0] * dist;
		float z = cameraPos[2] + cameraFront[2] * dist;

		if (dist > 0)
			carPos.push_back(glm::vec3(x, 0.0f, z));
			carRot.push_back(false);
	}
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos;
	lastX = xpos;
	lastY = ypos;

	float sensitivity = 0.2;
	xoffset *= sensitivity;
	yoffset *= sensitivity;

	yaw += xoffset;
	pitch += yoffset;

	if (pitch > 89.0f)
		pitch = 89.0f;
	if (pitch < -89.0f)
		pitch = -89.0f;

	glm::vec3 front;
	front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
	front.y = sin(glm::radians(pitch));
	front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));

	cameraFront = glm::normalize(front);
}

void create_roadmap()
{
	/* initialize random seed: */
	srand(time(NULL));

	// Random positions
	//std::vector<glm::vec3> points;
	for (int i = 0; i < numNewPos; i++)
	{
		float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float x = (r - 0.5f) * mapSize;
		r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float z = (r - 0.5f) * mapSize;
		points.push_back(glm::vec3(x, 0.0f, z));
	}

	for (int i = 0; i < agentPos.size(); i++)
	{
		startIndices[i] = points.size();
		points.push_back(agentPos[i]);
		goalIndices[i] = points.size();
		points.push_back(agentGoals[i]);
	}

	int numNodes = points.size();

	//std::vector<unsigned int> edges[numNewPos + numAgents];
	//std::vector<unsigned int> edgeIndices; // For drawing roadmap
	for (int i = 0; i < numNodes; i++)
	{
		// For each point connect to every point with line of sight
		for (int j = 0; j < numNodes; j++)
		{
			if (i != j)
			{
				glm::vec2 p1 = glm::vec2(points[i][0], points[i][2]);
				glm::vec2 p2 = glm::vec2(points[j][0], points[j][2]);
				if (!collidesWithObs(p1,p2))
				{
					edges[i].push_back(j);
					edgeIndices.push_back(i);
					edgeIndices.push_back(j);
				}
			}
		}
	}


	// Now make paths
	//A*
	for (int agent = 0; agent < agentPos.size(); agent++)
	{
		int start = startIndices[agent];
		int goal = goalIndices[agent];
		
		std::vector<unsigned int> fringe; //Known nodes not yet explored
		std::vector<unsigned int> explored; //Nodes already explored
		unsigned int* cameFrom = new unsigned int[numNodes]; // For each node the best node to get their from
		float* gVal = new float[numNodes]; // Cost of getting to each node
		float* fVal = new float[numNodes]; // Cost of getting to each node plus distance to goal
		for (int i = 0; i < numNodes; i++)
		{
			gVal[i] = INFINITY;
			fVal[i] = INFINITY;
		}
		gVal[start] = 0.0f;
		fVal[start] = glm::length(points[start] - points[goal]);
		fringe.push_back(start);

		unsigned int current = start;
		while (current != goal && fringe.size() > 0)
		{
			// First get lowest cost node in fringe - we will explore that one
			float lowestF = INFINITY;
			unsigned int lowestFIndex = 0;
			for (int i = 0; i < fringe.size(); i++)
			{ // For each node in fringe
				if (fVal[fringe[i]] < lowestF)
				{
					lowestF = fVal[fringe[i]];
					lowestFIndex = i;
					current = fringe[i];
				}
			}

			if (lowestF != INFINITY)
			{
				// Explore current node
				fringe.erase(fringe.begin() + lowestFIndex); // Remove from fringe
				explored.push_back(current); // Add to explored

				// For each neighbor of current node
				for (int i = 0; i < edges[current].size(); i++)
				{
					unsigned int lookingAt = edges[current][i];

					// If it has not already been explored
					bool hasBeenExplored = false;
					for (int j = 0; j < explored.size(); j++)
					{
						if (explored[j] == lookingAt)
						{
							hasBeenExplored = true;
						}
					}
					float pathLength = gVal[current] + glm::length(points[lookingAt] - points[current]);
					bool isInFringe = false;
					bool hasBetterPath = false;
					for (int j = 0; j < fringe.size(); j++)
					{
						if (fringe[j] == lookingAt)
						{
							isInFringe = true;
							if (gVal[lookingAt] < pathLength)
							{
								hasBetterPath = true;
								break;
							}
						}
					}

					if (!hasBeenExplored && !isInFringe)
					{
						// Add it if it isn't in the fringe
						fringe.push_back(lookingAt);

						cameFrom[lookingAt] = current;
						gVal[lookingAt] = pathLength;
						if (aStar)
							fVal[lookingAt] = 1.0f * pathLength + 1.0f * glm::length(points[lookingAt] - points[goal]);
						else
							fVal[lookingAt] = pathLength;
					}
					// If there isn't already a better path
					else if (isInFringe && !hasBetterPath)
					{
						cameFrom[lookingAt] = current;
						gVal[lookingAt] = pathLength;
						if (aStar)
							fVal[lookingAt] = 1.0f * pathLength + 1.0f * glm::length(points[lookingAt] - points[goal]);
						else
							fVal[lookingAt] = pathLength;
					}
				}
			}

		}

		//A* is done, build path
		std::vector<unsigned int> path;
		current = goal;
		while (current != start)
		{
			path.push_back(current);
			current = cameFrom[current];
		}
		path.push_back(start);
		//Now just pop path to get next point on path'

		nextPathPoint[agent] = points[path.back()];
		path.pop_back();
		paths[agent] = path;


		delete[] cameFrom;
		delete[] gVal;
		delete[] fVal;
	}
}

void addAgent(glm::vec3 start, glm::vec3 goal)
{
	agentPos.push_back(start);
	agentVel.push_back(glm::vec3(0.0f));
	forceAccum.push_back(glm::vec3(0.0f));
	agentGoals.push_back(goal);
	nextPathPoint.push_back(glm::vec3(0.0f));
	goalIndices.push_back(0);
	startIndices.push_back(0);
}

bool collidesWithObs(glm::vec2 point1, glm::vec2 point2)
{
	bool LOS = true;

	// Check for barrels
	for (int k = 0; k < barrelPos.size(); k++)
	{
		glm::vec2 obsPos = glm::vec2(barrelPos[k][0], barrelPos[k][2]);

		glm::vec2 line = point2 - point1;
		glm::vec2 pointToObs = obsPos - point1;
		float dot = glm::dot(pointToObs, glm::normalize(line));
		glm::vec2 nearestPoint = point1 + glm::normalize(line) * dot;

		bool point1InObs = glm::length(obsPos - point1) < barrelRadCoord;
		bool point2InObs = glm::length(obsPos - point2) < barrelRadCoord;
		bool onSegment = (dot > 0 && dot < glm::length(line));
		bool inObs(glm::length(obsPos - nearestPoint) < barrelRadCoord);

		if (point1InObs || point2InObs || (onSegment && inObs))
		{
			// No line of sight
			return true;
		}
	}

	//*
	// Check for cars
	for (int k = 0; k < carPos.size(); k++)
	{
		// See if line between points crosses any edges of rectangle and if points lie within rectangle
		//if (!carRot[k])
		{
			//*
			bool p1inX = (point1[0] > carPos[k][0] - (carX / 2)) && (point1[0] < carPos[k][0] + (carX / 2));
			bool p2inX = (point2[0] > carPos[k][0] - (carX / 2)) && (point2[0] < carPos[k][0] + (carX / 2));
			bool p1inZ = (point1[1] > carPos[k][2] - (carZ / 2)) && (point1[1] < carPos[k][2] + (carZ / 2));
			bool p2inZ = (point2[1] > carPos[k][2] - (carZ / 2)) && (point2[1] < carPos[k][2] + (carZ / 2));

			bool inside = (p1inX && p1inZ && p2inX && p2inZ);
			if (inside)
			{
				return true;
			}
			else
			{
				glm::vec2 c1 = glm::vec2(carPos[k][0], carPos[k][2]) + glm::vec2(-carX+agentRad / 2.0f, -carZ + agentRad / 2.0f);
				glm::vec2 c2 = glm::vec2(carPos[k][0], carPos[k][2]) + glm::vec2(carX + agentRad / 2.0f, -carZ + agentRad / 2.0f);
				glm::vec2 c3 = glm::vec2(carPos[k][0], carPos[k][2]) + glm::vec2(-carX + agentRad / 2.0f, carZ + agentRad / 2.0f);
				glm::vec2 c4 = glm::vec2(carPos[k][0], carPos[k][2]) + glm::vec2(carX + agentRad / 2.0f, carZ + agentRad / 2.0f);
				if (lineIntersection(point1, point2, c1, c2) || lineIntersection(point1, point2, c1, c3)
					|| lineIntersection(point1, point2, c4, c2) || lineIntersection(point1, point2, c4, c3))
				{
					//cout << "Points: " << point1[0] << "," << point1[1] << " and " << point2[0] << "," << point2[1] << endl;
					return true;
				}
			}
			//*/

			/*
			//Approximate car as 3 circles
				//Center
			glm::vec2 obsPos = glm::vec2(carPos[k][0], carPos[k][2]);
			glm::vec2 line = point2 - point1;
			glm::vec2 pointToObs = obsPos - point1;
			float dot = glm::dot(pointToObs, glm::normalize(line));
			glm::vec2 nearestPoint = point1 + glm::normalize(line) * dot;
			bool point1InObs = glm::length(obsPos - point1) < carZ + agentRad;
			bool point2InObs = glm::length(obsPos - point2) < carZ + agentRad;
			bool onSegment = (dot > 0 && dot < glm::length(line));
			bool inObs = (glm::length(obsPos - nearestPoint) < carZ + agentRad);
			if (point1InObs || point2InObs || (onSegment && inObs))
			{
				// No line of sight
				return true;
			}
				//Front
			obsPos = glm::vec2(carPos[k][0]+carX/2, carPos[k][2]);
			line = point2 - point1;
			pointToObs = obsPos - point1;
			 dot = glm::dot(pointToObs, glm::normalize(line));
			nearestPoint = point1 + glm::normalize(line) * dot;
			point1InObs = glm::length(obsPos - point1) < carZ + agentRad;
			point2InObs = glm::length(obsPos - point2) < carZ + agentRad;
			onSegment = (dot > 0 && dot < glm::length(line));
			inObs = (glm::length(obsPos - nearestPoint) < carZ + agentRad);
			if (point1InObs || point2InObs || (onSegment && inObs))
			{
				// No line of sight
				return true;
			}
				//Back
			obsPos = glm::vec2(carPos[k][0]-carX/2, carPos[k][2]);
			line = point2 - point1;
			pointToObs = obsPos - point1;
			dot = glm::dot(pointToObs, glm::normalize(line));
			nearestPoint = point1 + glm::normalize(line) * dot;
			point1InObs = glm::length(obsPos - point1) < carZ + agentRad;
			point2InObs = glm::length(obsPos - point2) < carZ + agentRad;
			onSegment = (dot > 0 && dot < glm::length(line));
			inObs = (glm::length(obsPos - nearestPoint) < carZ + agentRad);
			if (point1InObs || point2InObs || (onSegment && inObs))
			{
				// No line of sight
				return true;
			}
			//*/
		}
		//else
		{

		}
	}
	//*/

	// If we havent returned by now then there are no violations
	return false;
}

bool ccw(glm::vec2 a, glm::vec2 b, glm::vec2 c) //Determines if a,b,c are counterclockwise rotated
{
	return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0]);
}

bool lineIntersection(glm::vec2 p1, glm::vec2 p2, glm::vec2 q1, glm::vec2 q2)
{
	return (ccw(p1, q1, q2) != ccw(p2, q1, q2)) && (ccw(p1, p2, q1) != ccw(p1, p2, q2));
}