//|___________________________________________________________________
//!
//! \file plane1_base.cpp
//!
//! \brief Base source code for the first plane assignment.
//!
//! Author: Mores Prachyabrued.
//!
//! Keyboard controls:
//!   s   = moves the plane forward
//!   f   = moves the plane backward
//!	  x   = pitches the plane (+ X-rot)
//!   w   = pitches the plane (- X-rot)
//!	  d   = yaws the plane (+ Y-rot)
//!   a   = yaws the plane (- Y-rot)
//!   e   = rolls the plane (+ Z-rot)
//!	  q   = rolls the plane (- Z-rot)
//!
//!   k   = moves the camera forward
//!   ;   = moves the camera backward
//!   ,   = pitches the camera (+ X-rot)
//!   i   = pitches the camera (- X-rot)
//!   l   = yaws the camera (+ Y-rot)
//!   j   = yaws the camera (- Y-rot)
//!	  u   = rolls the camera (+ Z-rot)
//!	  o   = rolls the camera (- Z-rot)
//!
//! TODO: Extend the code to satisfy the requirements given in the assignment handout
//!
//! Note: Good programmer uses good comments! :)
//|___________________________________________________________________

//|___________________
//|
//| Includes
//|___________________

#include <math.h>

#include <gmtl/gmtl.h>

#include <GL/glut.h>

//|___________________
//|
//| Constants
//|___________________

// Plane dimensions
const float P_WIDTH = 1.5;
const float P_LENGTH = 1.5;
const float P_HEIGHT = 1.5;

// Camera's view frustum 
const float CAM_FOV = 60.0f;     // Field of view in degs

//|___________________
//|
//| Global Variables
//|___________________

// Track window dimensions, initialized to 800x600
int w_width = 800;
int w_height = 600;

// Plane pose (position & orientation)
gmtl::Matrix44f plane_pose; // T, as defined in the handout, initialized to IDENTITY by default

// Camera pose
gmtl::Matrix44f cam_pose;   // C, as defined in the handout
gmtl::Matrix44f view_mat;   // View transform is C^-1 (inverse of the camera transform C)

// fixed top-down camera
gmtl::Matrix44f cam_pose_fixed; // F, as defined in the handout
gmtl::Matrix44f view_mat_fixed; // view transform is F^-1 (inverse of the fixed topdown camera transform F)

// Transformation matrices applied to plane and camera poses
gmtl::Matrix44f ztransp_mat;
gmtl::Matrix44f ztransn_mat;
gmtl::Matrix44f zrotp_mat; // positive
gmtl::Matrix44f zrotn_mat; // negative
gmtl::Matrix44f yrotp_mat; // positive
gmtl::Matrix44f yrotn_mat; // negative
gmtl::Matrix44f xrotp_mat; // positive
gmtl::Matrix44f xrotn_mat; // negative

// preset colours
float colour_brown[3] = {0.45f, 0.32f, 0.22f};
float colour_lime_green[3] = {0.35f, 0.47f, 0.10f};
float colour_light_lime_green[3] = {0.45f, 0.57f, 0.20f};
float colour_dark_gray[3] = {0.2f, 0.2f, 0.2f};
// rgb(172, 92, 92)
float colour_light_pink[3] = {0.87f, 0.66f, 0.66f};


//|___________________
//|
//| Function Prototypes
//|___________________

void InitMatrices();
void InitGL(void);
void DisplayFunc(void);
void KeyboardFunc(unsigned char key, int x, int y);
void ReshapeFunc(int w, int h);
void DrawCoordinateFrame(const float l);
void DrawObject(const float width, const float length, const float height);

//|____________________________________________________________________
//|
//| Function: InitMatrices
//|
//! \param None.
//! \return None.
//!
//! Initializes all the matrices
//|____________________________________________________________________

void InitMatrices()
{
	const float TRANS_AMOUNT = 1.0f;
	const float ROT_AMOUNT = gmtl::Math::deg2Rad(5.0f); // specified in degs, but get converted to radians

	const float COSTHETA = cos(ROT_AMOUNT);
	const float SINTHETA = sin(ROT_AMOUNT);

	// Positive Z-translation
	ztransp_mat.set(1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, TRANS_AMOUNT,
		0, 0, 0, 1);
	ztransp_mat.setState(gmtl::Matrix44f::TRANS);

	// Negative Z-translation
	gmtl::invert(ztransn_mat, ztransp_mat);

	// Positive Z-rotation (roll)
	zrotp_mat.set(COSTHETA, -SINTHETA, 0, 0,
		SINTHETA, COSTHETA, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);
	zrotp_mat.setState(gmtl::Matrix44f::ORTHOGONAL);

	// Negative Z-rotation (roll)
	gmtl::invert(zrotn_mat, zrotp_mat);

	// Positive Y-rotation (yaw)
	yrotp_mat.set(COSTHETA, 0, SINTHETA, 0,
		0, 1, 0, 0,
		-SINTHETA, 0, COSTHETA, 0,
		0, 0, 0, 1);
	yrotp_mat.setState(gmtl::Matrix44f::ORTHOGONAL);

	// Negative Y-rotation (yaw)
	gmtl::invert(yrotn_mat, yrotp_mat);

	// Positive X-rotation (pitch)
	xrotp_mat.set(1, 0, 0, 0,
		0, COSTHETA, -SINTHETA, 0,
		0, SINTHETA, COSTHETA, 0,
		0, 0, 0, 1);
	xrotp_mat.setState(gmtl::Matrix44f::ORTHOGONAL);

	// Negative X-rotation (pitch)
	gmtl::invert(xrotn_mat, xrotp_mat);

	// Inits plane pose (rigid)
	plane_pose.set(1, 0, 0, 1.0f,
		0, 1, 0, 0.0f,
		0, 0, 1, 4.0f,
		0, 0, 0, 1.0f);
	plane_pose.setState(gmtl::Matrix44f::AFFINE);     // AFFINE because the plane pose can contain both translation and rotation         

	// Inits camera pose and view transform (rigid)
	cam_pose.set(1, 0, 0, 2.0f,
		0, 1, 0, 1.0f,
		0, 0, 1, 15.0f,
		0, 0, 0, 1.0f);
	cam_pose.setState(gmtl::Matrix44f::AFFINE);
	gmtl::invert(view_mat, cam_pose);                 // View transform is the inverse of the camera pose

	gmtl::Matrix44f rot_mat, trans_mat;
	rot_mat.set(1, 0, 0, 0,				// X rot by -90 degrees
		0, 0, 1, 0,
		0, -1, 0, 0,
		0, 0, 0, 1);
	rot_mat.setState(gmtl::Matrix44f::ORTHOGONAL);
	trans_mat.set(1, 0, 0, 0,			// translate y by 20
		0, 1, 0, 30,
		0, 0, 1, 0,
		0, 0, 0, 1);
	trans_mat.setState(gmtl::Matrix44f::TRANS);
	cam_pose_fixed = trans_mat * rot_mat;
	gmtl::invert(view_mat_fixed, cam_pose_fixed);		// view transform is the inverse of the camera pose
}

//|____________________________________________________________________
//|
//| Function: InitGL
//|
//! \param None.
//! \return None.
//!
//! OpenGL initializations
//|____________________________________________________________________

void InitGL(void)
{
	glClearColor(0.7f, 0.8f, 0.7f, 1.0f);
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);
}

//|____________________________________________________________________
//|
//| Function: DisplayFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT display callback function: called for every redraw event.
//|____________________________________________________________________

void DisplayFunc(void)
{
	// Modelview matrix
	gmtl::Matrix44f modelview_mat;        // M, as defined in the handout

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//|____________________________________________________________________
	//|
	//| Viewport 1 rendering: shows the moving camera's view
	//|____________________________________________________________________

	// setup viewport transformation matrix in the pipeline
	glViewport(0, 0, (GLsizei)w_width / 2, (GLsizei)w_height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(CAM_FOV, (float)w_width / (2 * w_height), 0.1f, 100.0f);     // Check MSDN: google "gluPerspective msdn"

	// Approach1
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();                          // A good practice for beginner

	// Draws world coordinate frame
	modelview_mat = view_mat;                  // M = C^-1
	glLoadMatrixf(modelview_mat.mData); // load input matrix into the target (modelview) matrix
	DrawCoordinateFrame(10);

	// Draws plane and its local frame
	modelview_mat *= plane_pose;               // M = C^-1 * T
	glLoadMatrixf(modelview_mat.mData);
	DrawObject(P_WIDTH, P_LENGTH, P_HEIGHT);
	DrawCoordinateFrame(3);

	/*
	  // Approach 2 (gives the same results as the approach 1)
	  glMatrixMode(GL_MODELVIEW);

	  // Draws world coordinate frame
	  glLoadMatrixf(view_mat.mData);             // M = C^-1
	  DrawCoordinateFrame(10);

	  // Draws plane and its local frame
	  glMultMatrixf(plane_pose.mData);           // M = C^-1 * T (OpenGL calls build transforms in left-to-right order)
	  DrawPlane(P_WIDTH, P_LENGTH, P_HEIGHT);
	  DrawCoordinateFrame(3);
	*/

	//|____________________________________________________________________
	//|
	//| TODO: Viewport 2 rendering: shows the fixed top-down view
	//|____________________________________________________________________

	glViewport(w_width / 2, 0, (GLsizei)w_width / 2, (GLsizei)w_height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(CAM_FOV, (float)w_width / (2 * w_height), 0.1f, 100.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// draws world coordinate frame
	modelview_mat = view_mat_fixed;			// M = F^-1
	glLoadMatrixf(modelview_mat.mData);
	DrawCoordinateFrame(10);

	// Draws plane and its local frame
	modelview_mat *= plane_pose;               // M = F^-1 * T
	glLoadMatrixf(modelview_mat.mData);
	DrawObject(P_WIDTH, P_LENGTH, P_HEIGHT);
	DrawCoordinateFrame(3);

	// Draws movable camera
	modelview_mat = view_mat_fixed * cam_pose;   // M = F^-1 * C
	glLoadMatrixf(modelview_mat.mData);
	DrawCoordinateFrame(1);

	glFlush();
}

//|____________________________________________________________________
//|
//| Function: KeyboardFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT keyboard callback function: called for every key press event.
//|____________________________________________________________________

void KeyboardFunc(unsigned char key, int x, int y)
{
	switch (key) {
		//|____________________________________________________________________
		//|
		//| Plane controls
		//|____________________________________________________________________

	case 's': // Forward translation of the plane (positive Z-translation)
		plane_pose = plane_pose * ztransp_mat;
		break;
	case 'f': // Backward translation of the plane
		plane_pose = plane_pose * ztransn_mat;
		break;

		// PITCH //////////////////////////
	case 'x': // Pitches the plane (+ X-rot)
		plane_pose = plane_pose * xrotp_mat;
		break;
	case 'w': // Pitches the plane (- X-rot)
		plane_pose = plane_pose * xrotn_mat;
		break;

		// YAW //////////////////////////
	case 'd': // Yaws the plane (+ Y-rot)
		plane_pose = plane_pose * yrotp_mat;
		break;
	case 'a': // Yaws the plane (- Y-rot)
		plane_pose = plane_pose * yrotn_mat;
		break;

		// ROLL //////////////////////////
	case 'e': // Rolls the plane (+ Z-rot)
		plane_pose = plane_pose * zrotp_mat;
		break;
	case 'q': // Rolls the plane (- Z-rot)
		plane_pose = plane_pose * zrotn_mat;
		break;




		// TODO: Add the remaining controls/transforms        

	//|____________________________________________________________________
	//|
	//| Camera controls
	//|____________________________________________________________________

	case 'k': // Forward translation of the camera (negative Z-translation - cameras looks in its (local) -Z direction)
		cam_pose = cam_pose * ztransn_mat;
		break;
	case ';': // Backward translation of the camera
		cam_pose = cam_pose * ztransp_mat;
		break;

		// TODO: Add the remaining controls
			// PITCH //////////////////////////
	case ',': // Pitches the camera (+ X-rot)
		cam_pose = cam_pose * xrotp_mat;
		break;
	case 'i': // Pitches the camera (- X-rot)
		cam_pose = cam_pose * xrotn_mat;
		break;

		// YAW //////////////////////////
	case 'l': // Yaws the camera (+ Y-rot)
		cam_pose = cam_pose * yrotp_mat;
		break;
	case 'j': // Yaws the camera (- Y-rot)
		cam_pose = cam_pose * yrotn_mat;
		break;

		// ROLL //////////////////////////
	case 'u': // Rolls the camera (+ Z-rot)
		cam_pose = cam_pose * zrotp_mat;
		break;
	case 'o': // Rolls the camera (- Z-rot)
		cam_pose = cam_pose * zrotn_mat;
		break;
	}

	gmtl::invert(view_mat, cam_pose);       // Updates view transform to reflect the change in camera transform
	glutPostRedisplay();                    // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: ReshapeFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT reshape callback function: called everytime the window is resized.
//|____________________________________________________________________

void ReshapeFunc(int w, int h)
{
	// Track the current window dimensions
	w_width = w;
	w_height = h;
}

//|____________________________________________________________________
//|
//| Function: DrawCoordinateFrame
//|
//! \param l      [in] length of the three axes.
//! \return None.
//!
//! Draws coordinate frame consisting of the three principal axes.
//|____________________________________________________________________

void DrawCoordinateFrame(const float l)
{
	glBegin(GL_LINES);
	// X axis is red
	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(l, 0.0f, 0.0f);

	// Y axis is green
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, l, 0.0f);

	// Z axis is blue
	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, l);
	glEnd();
}

//|____________________________________________________________________
//|
//| Function: DrawPlane
//|
//! \param width       [in] Width  of the plane.
//! \param length      [in] Length of the plane.
//! \param height      [in] Height of the plane.
//! \return None.
//!
//! Draws the plane.
//|____________________________________________________________________

void drawCube(const float width, const float length, const float height, const float colours[3]) {
	float w2 = width / 2;
	float h2 = height / 2;
	float l2 = length / 2;

	float colours_copy[3] = {colours[0], colours[1], colours[2]}; // modify the copy only (arrays are passed by reference)
	float c_delta = 0.05f; // for adding subtle shadow

	glBegin(GL_QUADS);
	// front
	glColor3f(colours_copy[0], colours_copy[1], colours_copy[2]);
	glVertex3f(w2, h2, -l2);
	glVertex3f(-w2, h2, -l2);
	glVertex3f(-w2, -h2, -l2);
	glVertex3f(w2, -h2, -l2);

	colours_copy[0] += c_delta;
	colours_copy[1] += c_delta;
	colours_copy[2] += c_delta;

	// right
	glColor3f(colours_copy[0], colours_copy[1], colours_copy[2]);
	glVertex3f(w2, h2, -l2);
	glVertex3f(w2, h2, l2);
	glVertex3f(w2, -h2, l2);
	glVertex3f(w2, -h2, -l2);

	colours_copy[0] += c_delta;
	colours_copy[1] += c_delta;
	colours_copy[2] += c_delta;

	// top
	glColor3f(colours_copy[0], colours_copy[1], colours_copy[2]);
	glVertex3f(w2, h2, l2);
	glVertex3f(-w2, h2, l2);
	glVertex3f(-w2, h2, -l2);
	glVertex3f(w2, h2, -l2);

	colours_copy[0] += c_delta;
	colours_copy[1] += c_delta;
	colours_copy[2] += c_delta;

	// bottom
	glColor3f(colours_copy[0], colours_copy[1], colours_copy[2]);
	glVertex3f(w2, -h2, -l2);
	glVertex3f(-w2, -h2, -l2);
	glVertex3f(-w2, -h2, l2);
	glVertex3f(w2, -h2, l2);

	colours_copy[0] += c_delta;
	colours_copy[1] += c_delta;
	colours_copy[2] += c_delta;

	// back
	glColor3f(colours_copy[0], colours_copy[1], colours_copy[2]);
	glVertex3f(-w2, h2, l2);
	glVertex3f(w2, h2, l2);
	glVertex3f(w2, -h2, l2);
	glVertex3f(-w2, -h2, l2);

	colours_copy[0] += c_delta;
	colours_copy[1] += c_delta;
	colours_copy[2] += c_delta;

	// left
	glColor3f(colours_copy[0], colours_copy[1], colours_copy[2]);
	glVertex3f(-w2, h2, -l2);
	glVertex3f(-w2, h2, l2);
	glVertex3f(-w2, -h2, l2);
	glVertex3f(-w2, -h2, -l2);
	glEnd();
}

void DrawObject(const float width, const float length, const float height)
{
	// shell
	drawCube(1.7f*width, 2.0f*length, 0.7f*height, colour_brown);

	// head
	glPushMatrix();
	glTranslatef(0.0f*width, -0.1f*height, 0.8f*length);
	drawCube(0.7f*width, 0.7f*length, 0.45f*height, colour_lime_green);
	glPopMatrix();

	// left eye
	glPushMatrix();
	glTranslatef(-0.27f*width, -0.20f*height, 1.15f*length);
	drawCube(0.11f*width, 0.11f*length, 0.11f*height, colour_dark_gray);
	glPopMatrix();

	// right eye
	glPushMatrix();
	glTranslatef(0.27f*width, -0.20f*height, 1.15f*length);
	drawCube(0.11f*width, 0.11f*length, 0.11f*height, colour_dark_gray);
	glPopMatrix();

	// tail (tilted a bit)
	glPushMatrix();
	glTranslatef(0.0f*width, -0.2f*height, -0.9f*length);
	glRotatef(30.0f, 0.0f*width, 1.0f*height, 0.0f*length);
	drawCube(0.2f*width, 0.8f*length, 0.2f, colour_lime_green);
	glPopMatrix();

	// front left leg
	glPushMatrix();
	glTranslatef(0.9f*width, -0.26f*height, 0.6f*length);
	drawCube(1.6f*width, 0.6f*length, 0.15f*height, colour_light_lime_green);
	glPopMatrix();

	// front right leg
	glPushMatrix();
	glTranslatef(-0.9f*width, -0.26f*height, 0.6f*length);
	drawCube(1.6f*width, 0.6f*length, 0.15f*height, colour_light_lime_green);
	glPopMatrix();

	// back left leg
	glPushMatrix();
	glTranslatef(0.9f*width, -0.28f*height, -0.6f*length);
	drawCube(0.8f*width, 0.6f, 0.15f, colour_light_lime_green);
	glPopMatrix();

	// back right leg
	glPushMatrix();
	glTranslatef(-0.9f*width, -0.28f*height, -0.6f*length);
	drawCube(0.8f*width, 0.6f, 0.15f, colour_light_lime_green);
	glPopMatrix();
}

//|____________________________________________________________________
//|
//| Function: main
//|
//! \param None.
//! \return None.
//!
//! Program entry point
//|____________________________________________________________________

int main(int argc, char** argv)
{
	InitMatrices();

	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(w_width, w_height);

	glutCreateWindow("Plane Episode 1");

	glutDisplayFunc(DisplayFunc);
	glutReshapeFunc(ReshapeFunc);
	glutKeyboardFunc(KeyboardFunc);

	InitGL();

	glutMainLoop();

	return 0;
}