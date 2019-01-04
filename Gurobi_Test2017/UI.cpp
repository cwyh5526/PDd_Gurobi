//============================================================================
// File: gl_ui.cpp
// Date: 2018.12.06
// Name: Jisu Kim
//
// glui for deformable Penetaration Depth
//============================================================================

#include "UI.h"
#include "DefPD.h"

#define OFF 0
#define ON 1

/********** User IDs for callbacks ********/

#define INPUT_STATIC_TET_ID		200
#define INPUT_REST_TET_ID		201

#define OPTIMIZE_ID				250 //opt button

#define STATIC_TET_RENDER	 300	//
#define REST_TET_RENDER		 301
#define OPTIMAL_TET_RENDER	 302
#define DEFORM_TET_RENDER	 303
#define GROUND_RENDER		 304

//optimization object;
DefPD *defPD;

/********** rendering variable **********/
int   main_window;
float scale = 1.0;
float view_rotate[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
float obj_pos[] = { 0.0, 0.0, 0.0 };

float xy_aspect;
int   last_x, last_y;
float rotationX = 0.0, rotationY = 0.0;


/** These are the live variables passed into GLUI ***/
// input from user
tet staticTet;
tet restTet;
tet optimalTet;
optResults allResults;

float totalOptTime;
float minOptValue;
int minOptIndex;

//rendering option
int   render_static_option = ON;
int   render_rest_option = ON;
int   render_optimal_option = ON;
int   render_ground_option = ON;
int	  renderDeform[45] = { 0 };


/** Pointers to the windows and some of the controls we'll create **/
GLUI *glui, *glui2;

//output
GLUI_EditText   *edit_sTet[4][3];
GLUI_EditText   *edit_rTet[4][3];
GLUI_EditText   *edit_pTet[4][3];
GLUI_EditText	*text_optPair;
GLUI_EditText	*text_optValue;
GLUI_EditText	*text_optTime;

GLUI_Checkbox	*check_deform[45];
/********** Miscellaneous global variables **********/

GLfloat light0_ambient[] = { 0.1f, 0.1f, 0.3f, 1.0f };
GLfloat light0_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
GLfloat light0_position[] = { .5f, .5f, 1.0f, 0.0f };

GLfloat lights_rotation[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

/**************************************** control_cb() *******************/
/* GLUI control callback                                                 */

void control_cb(int control)
{

	if (control == OPTIMIZE_ID) {
				
		//optimize
		cout << "default? Y/N" << endl;
		char ans = 'Y';
		cin >> ans;
		if (ans == 'Y' || ans == 'y') {
			defPD->initDefault();
		}
		else {
			defPD->init(staticTet, restTet);
		}
		defPD->resolvePenetration();

		//results
		staticTet = defPD->getSTet();
		restTet = defPD->getRTet();
		optimalTet = defPD->getPTet();

		minOptIndex = defPD->getMinOptIndex();
		minOptValue = defPD->getPD();
		totalOptTime = defPD->getOptTime();
		allResults = defPD->getPTetAll();

		for (int v = 0; v < 4; v++) {
			edit_sTet[v][0]->set_float_val(staticTet.vertex[v].x);
			edit_sTet[v][1]->set_float_val(staticTet.vertex[v].y);
			edit_sTet[v][2]->set_float_val(staticTet.vertex[v].z);

			edit_rTet[v][0]->set_float_val(restTet.vertex[v].x);
			edit_rTet[v][1]->set_float_val(restTet.vertex[v].y);
			edit_rTet[v][2]->set_float_val(restTet.vertex[v].z);

			edit_pTet[v][0]->set_float_val(optimalTet.vertex[v].x);
			edit_pTet[v][1]->set_float_val(optimalTet.vertex[v].y);
			edit_pTet[v][2]->set_float_val(optimalTet.vertex[v].z);
		}
		if (minOptIndex < 4) {
			text_optPair->set_text("Static Face" + to_string(minOptIndex));
		}
		else if (minOptIndex < 8) {
			text_optPair->set_text("Deforming Face" + to_string(minOptIndex-4));

		}
		else if (minOptIndex < 44) {
			string str = "sE" + to_string((minOptIndex - 8) / 6)+ "dE" + to_string((minOptIndex - 8) % 6);
			text_optPair->set_text(str);

		}
		
		text_optValue->set_float_val(minOptValue);
		text_optTime->set_float_val(totalOptTime);

		defPD->printResult(minOptIndex);
		glutPostRedisplay();


	}
	if ((control == REST_TET_RENDER)|| (control == STATIC_TET_RENDER)|| (control == OPTIMAL_TET_RENDER)||(control ==GROUND_RENDER)) {
		
		glutPostRedisplay();

	}
	if (control == DEFORM_TET_RENDER) {
		if (renderDeform[44] == ON) { //if Off checkboxed is checked
			for (int i = 0; i < 44; i++) {

				renderDeform[i] = OFF; //remove all checkboxses
				check_deform[i]->set_int_val(OFF);
			}
		}
		
		cout << "Deform_tet_render : " << renderDeform[44] << endl;
		glutPostRedisplay();
	}
	
	//if()
	/*
	else {
	glDisable(GL_LIGHT0);
	light0_spinner->disable();
	}
	}
	else if (control == LIGHT1_ENABLED_ID) {
	if (light1_enabled) {
	glEnable(GL_LIGHT1);
	light1_spinner->enable();
	}
	else {
	glDisable(GL_LIGHT1);
	light1_spinner->disable();
	}
	}
	else if (control == LIGHT0_INTENSITY_ID) {
	float v[] = {
	light0_diffuse[0], light0_diffuse[1],
	light0_diffuse[2], light0_diffuse[3] };

	v[0] *= light0_intensity;
	v[1] *= light0_intensity;
	v[2] *= light0_intensity;

	glLightfv(GL_LIGHT0, GL_DIFFUSE, v);
	}
	*/
	/*else if (control == ENABLE_ID)
	{
	glui2->enable();
	}
	else if (control == DISABLE_ID)
	{
	glui2->disable();
	}
	else if (control == SHOW_ID)
	{
	glui2->show();
	}
	else if (control == HIDE_ID)
	{
	glui2->hide();
	}
	else if (control == NEWPOSITION_ID)
	{
	seed = rand();
	glutPostRedisplay();

	}*/
}

/**************************************** myGlutKeyboard() **********/

void myGlutKeyboard(unsigned char Key, int x, int y)
{
	switch (Key)
	{
	case 27:
	case 'q':
		exit(0);
		break;
	};

	glutPostRedisplay();
}


/***************************************** myGlutMenu() ***********/

void myGlutMenu(int value)
{
	myGlutKeyboard(value, 0, 0);
}


/***************************************** myGlutIdle() ***********/

void myGlutIdle(void)
{
	/* According to the GLUT specification, the current window is
	undefined during an idle callback.  So we need to explicitly change
	it if necessary */
	if (glutGetWindow() != main_window)
		glutSetWindow(main_window);

	/*  GLUI_Master.sync_live_all();  -- not needed - nothing to sync in this
	application  */

	glutPostRedisplay();
}

/***************************************** myGlutMouse() **********/

void myGlutMouse(int button, int button_state, int x, int y)
{
}


/***************************************** myGlutMotion() **********/

void myGlutMotion(int x, int y)
{
	glutPostRedisplay();
}

/**************************************** myGlutReshape() *************/

void myGlutReshape(int x, int y)
{
	int tx, ty, tw, th;
	GLUI_Master.get_viewport_area(&tx, &ty, &tw, &th);
	glViewport(tx, ty, tw, th);

	xy_aspect = (float)tw / (float)th;

	glutPostRedisplay();
}

//===================================================================================
//  create Tets with 4 vertices, Init Tet
//====================================================================================
void drawTet(tet t, float r, float g, float b, float alpha) {
	glPushMatrix();
	glTranslatef(0, 0, 0);
	//glLoadIdentity();
	glBegin(GL_TRIANGLES);
	for (int f = 0; f < 4; f++) {
		glColor4f(r/4.0f*(float)(3-f), g/4.0f*(float)(3-f), b/4.0f*(float)(3-f), alpha);
		for (int e = 0; e < 3; e++) {
			glVertex3f(t.face[f][e].x, t.face[f][e].y, t.face[f][e].z);
		}
	}
	glEnd();
	glPopMatrix();

}

/***************************************** myGlutDisplay() *****************/
void drawAxis() {
	glPushMatrix();
	glBegin(GL_LINES);
	glColor3f(0.f, 0.f, 0.f);
	//x axis
	glVertex3f(-10.f, 0.f, 0.f);
	glVertex3f(10.f, 0.f, 0.f);

	glVertex3f(0.f, -10.f, 0.f);
	glVertex3f(0.f, 10.f, 0.f);

	glVertex3f(0.f, 0.f, -10.f);
	glVertex3f(0.f, 0.f, 10.f);
	glEnd();
	glPopMatrix();
}
void drawGround() {
	glPushMatrix();

	glColor4f(0.7f, 0.7f, 0.7f, 0.5f);
	glBegin(GL_QUADS);
	glNormal3f(0, 1, 0);
	glVertex3f(-5, 0, 5);
	glVertex3f(5, 0, 5);
	glVertex3f(5, 0, -5);
	glVertex3f(-5, 0, -5);
	glEnd();
	glPopMatrix();

}
void myGlutDisplay(void)
{
	glClearColor(.9f, .9f, .9f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(45.0f, xy_aspect, 0.1f, 100);


	glMatrixMode(GL_MODELVIEW);

	glLoadIdentity();
	glMultMatrixf(lights_rotation);
	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);

	glLoadIdentity();

	glTranslatef(0.0, -1.0, -10.f);
	glTranslatef(obj_pos[0], obj_pos[1], -obj_pos[2]);
	glMultMatrixf(view_rotate);

	//glScalef(scale, scale, scale);
	glShadeModel(GL_FLAT);

	//===================================================================================
	//  Display
	//====================================================================================
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	drawAxis();
	//glTranslatef();


	if (renderDeform[44] == OFF) { //when off is not checked
		for (int i = 0; i < 44; i++) {
			if (renderDeform[i] == ON) {
				drawTet(allResults.pTets[i], 0.5f, 0.5f, 0.5f, 0.5f);
			}
		}
	}
	if (render_optimal_option == ON) {
		drawTet(optimalTet, 0.1, 0.1, 1.0, 0.3);
	}
	if (render_rest_option == ON) {
		drawTet(restTet, 0.1, 1.0, 0.1, 0.3);
	}
	if (render_static_option == ON) {
		drawTet(staticTet, 1.0, 0.1, 0.1, 0.3);
	}
	if (render_ground_option == ON) {
		drawGround();
	}

	glEnable(GL_LIGHTING);
	glutSwapBuffers();
}


/**************************************** main() ********************/

int main(int argc, char* argv[])
{
	/****************************************/
	/*   Initialize GLUT and create window  */
	/****************************************/

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowPosition(50, 50);
	glutInitWindowSize(1500, 900);

	main_window = glutCreateWindow("Collision Detection between Spheres");
	glutDisplayFunc(myGlutDisplay);
	GLUI_Master.set_glutReshapeFunc(myGlutReshape);
	GLUI_Master.set_glutKeyboardFunc(myGlutKeyboard);
	GLUI_Master.set_glutSpecialFunc(NULL);
	GLUI_Master.set_glutMouseFunc(myGlutMouse);
	glutMotionFunc(myGlutMotion);

	/****************************************/
	/*       Set up OpenGL lights           */
	/****************************************/

	glEnable(GL_LIGHTING);
	glEnable(GL_NORMALIZE);

	glEnable(GL_LIGHT0);

	glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);


	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	/****************************************/
	/*          Enable z-buferring          */
	/****************************************/

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);


	/****************************************/
	/*         Here's the GLUI code         */
	/****************************************/

	printf("GLUI version: %3.2f\n", GLUI_Master.get_version());

	/*** Create the side subwindow ***/
	glui = GLUI_Master.create_glui_subwindow(main_window,
		GLUI_SUBWINDOW_RIGHT);


	/****** Input panel ******/
	GLUI_Panel *in_out_panel = new GLUI_Panel(glui, "", GLUI_PANEL_NONE);

	GLUI_Rollout *input_panel = new GLUI_Rollout(in_out_panel, "Input", true);
	new GLUI_StaticText(input_panel, "Static Tetrahedron");
	GLUI_Panel *pan_s0 = new GLUI_Panel(input_panel, "s0", true);
	GLUI_Panel *pan_s1 = new GLUI_Panel(input_panel, "s1", true);
	GLUI_Panel *pan_s2 = new GLUI_Panel(input_panel, "s2", true);
	GLUI_Panel *pan_s3 = new GLUI_Panel(input_panel, "s3", true);

	edit_sTet[0][0] = new GLUI_EditText(pan_s0, "x:", &staticTet.vertex[0].x);
	edit_sTet[0][1] = new GLUI_EditText(pan_s0, "y:", &staticTet.vertex[0].y);
	edit_sTet[0][2] = new GLUI_EditText(pan_s0, "z:", &staticTet.vertex[0].z);
		 
	edit_sTet[1][0] = new GLUI_EditText(pan_s1, "x:", &staticTet.vertex[1].x);
	edit_sTet[1][1] = new GLUI_EditText(pan_s1, "y:", &staticTet.vertex[1].y);
	edit_sTet[1][2] = new GLUI_EditText(pan_s1, "z:", &staticTet.vertex[1].z);
		 
	edit_sTet[2][0] = new GLUI_EditText(pan_s2, "x:", &staticTet.vertex[2].x);
	edit_sTet[2][1] = new GLUI_EditText(pan_s2, "y:", &staticTet.vertex[2].y);
	edit_sTet[2][2] = new GLUI_EditText(pan_s2, "z:", &staticTet.vertex[2].z);
		 
	edit_sTet[3][0] = new GLUI_EditText(pan_s3, "x:", &staticTet.vertex[3].x);
	edit_sTet[3][1] = new GLUI_EditText(pan_s3, "y:", &staticTet.vertex[3].y);
	edit_sTet[3][2] = new GLUI_EditText(pan_s3, "z:", &staticTet.vertex[3].z);




	glui->add_column_to_panel(input_panel);

	new GLUI_StaticText(input_panel, "Rest Tetrahedron");
	GLUI_Panel *pan_r0 = new GLUI_Panel(input_panel, "r0", true);
	GLUI_Panel *pan_r1 = new GLUI_Panel(input_panel, "r1", true);
	GLUI_Panel *pan_r2 = new GLUI_Panel(input_panel, "r2", true);
	GLUI_Panel *pan_r3 = new GLUI_Panel(input_panel, "r3", true);


	edit_rTet[0][0] = new GLUI_EditText(pan_r0, "x:", &restTet.vertex[0].x);
	edit_rTet[0][1] = new GLUI_EditText(pan_r0, "y:", &restTet.vertex[0].y);
	edit_rTet[0][2] = new GLUI_EditText(pan_r0, "z:", &restTet.vertex[0].z);
		 
	edit_rTet[1][0] = new GLUI_EditText(pan_r1, "x:", &restTet.vertex[1].x);
	edit_rTet[1][1] = new GLUI_EditText(pan_r1, "y:", &restTet.vertex[1].y);
	edit_rTet[1][2] = new GLUI_EditText(pan_r1, "z:", &restTet.vertex[1].z);
		 
	edit_rTet[2][0] = new GLUI_EditText(pan_r2, "x:", &restTet.vertex[2].x);
	edit_rTet[2][1] = new GLUI_EditText(pan_r2, "y:", &restTet.vertex[2].y);
	edit_rTet[2][2] = new GLUI_EditText(pan_r2, "z:", &restTet.vertex[2].z);
		 
	edit_rTet[3][0] = new GLUI_EditText(pan_r3, "x:", &restTet.vertex[3].x);
	edit_rTet[3][1] = new GLUI_EditText(pan_r3, "y:", &restTet.vertex[3].y);
	edit_rTet[3][2] = new GLUI_EditText(pan_r3, "z:", &restTet.vertex[3].z);


	glui->add_column_to_panel(in_out_panel, false);

	//Optimze Button
	GLUI_Button *btn_opt = new GLUI_Button(glui, "Optimize", OPTIMIZE_ID, control_cb);;
	//btn_opt->set_alignment(GLUI_ALIGN_LEFT);

	/******* Output panel *********/

	GLUI_Rollout *output_panel = new GLUI_Rollout(in_out_panel, "Output", true);
	new GLUI_StaticText(output_panel, "Deformed Tetrahedron");
	GLUI_Panel *pan_p0 = new GLUI_Panel(output_panel, "p0", true);
	GLUI_Panel *pan_p1 = new GLUI_Panel(output_panel, "p1", true);
	GLUI_Panel *pan_p2 = new GLUI_Panel(output_panel, "p2", true);
	GLUI_Panel *pan_p3 = new GLUI_Panel(output_panel, "p3", true);


	edit_pTet[0][0] = new GLUI_EditText(pan_p0, "x:",&optimalTet.vertex[0].x);
	edit_pTet[0][1] = new GLUI_EditText(pan_p0, "y:",&optimalTet.vertex[0].y);
	edit_pTet[0][2] = new GLUI_EditText(pan_p0, "z:",&optimalTet.vertex[0].z);

	edit_pTet[1][0] = new GLUI_EditText(pan_p1, "x:", &optimalTet.vertex[1].x);
	edit_pTet[1][1] = new GLUI_EditText(pan_p1, "y:", &optimalTet.vertex[1].y);
	edit_pTet[1][2] = new GLUI_EditText(pan_p1, "z:", &optimalTet.vertex[1].z);

	edit_pTet[2][0] = new GLUI_EditText(pan_p2, "x:", &optimalTet.vertex[2].x);
	edit_pTet[2][1] = new GLUI_EditText(pan_p2, "y:", &optimalTet.vertex[2].y);
	edit_pTet[2][2] = new GLUI_EditText(pan_p2, "z:", &optimalTet.vertex[2].z);

	edit_pTet[3][0] = new GLUI_EditText(pan_p3, "x:", &optimalTet.vertex[3].x);
	edit_pTet[3][1] = new GLUI_EditText(pan_p3, "y:", &optimalTet.vertex[3].y);
	edit_pTet[3][2] = new GLUI_EditText(pan_p3, "z:", &optimalTet.vertex[3].z);

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++) {
			edit_sTet[i][j]->set_w(5);
			edit_rTet[i][j]->set_w(5);
			edit_pTet[i][j]->set_w(5);
			
		}
	}

	glui->add_column_to_panel(output_panel);

	new GLUI_StaticText(output_panel, "Optimization Results");
	new GLUI_StaticText(output_panel, "");
	text_optPair = new GLUI_EditText(output_panel, "Optimal Pair : ");
	text_optValue = new GLUI_EditText(output_panel, "Optimal Value: ", &minOptValue);
	text_optTime = new GLUI_EditText(output_panel, "Optimal Time : ", &totalOptTime);

	text_optValue->set_alignment(GLUI_ALIGN_RIGHT);
	text_optTime->set_alignment(GLUI_ALIGN_RIGHT);

	glui->add_separator();



	/****** Rendering Panel ******/

	GLUI_Rollout *rendering_panel = new GLUI_Rollout(glui, "Rendering Option", true);
	//GLUI_StaticText *text_rendering = new GLUI_StaticText(rendering_panel, "Select the pair that you want to see the result.");
	GLUI_Panel *rendering_panel_up = new GLUI_Panel(rendering_panel, "Optimal Result", false);
	GLUI_Panel *rendering_panel_down = new GLUI_Panel(rendering_panel, "All Deforming Result", true);

	GLUI_Panel *text_render_static = new GLUI_Panel(rendering_panel_up, "Static Tet rendering", true);
	GLUI_RadioGroup *render_static = new GLUI_RadioGroup(text_render_static, &render_static_option, STATIC_TET_RENDER, control_cb);
	new GLUI_RadioButton(render_static, "Off");
	new GLUI_RadioButton(render_static, "On");
	glui->add_column_to_panel(rendering_panel_up, false);

	GLUI_Panel *text_render_rest = new GLUI_Panel(rendering_panel_up, "Rest Tet rendering", true);
	GLUI_RadioGroup *render_rest = new GLUI_RadioGroup(text_render_rest, &render_rest_option, REST_TET_RENDER, control_cb);

	new GLUI_RadioButton(render_rest, "Off");
	new GLUI_RadioButton(render_rest, "On");
	glui->add_column_to_panel(rendering_panel_up, false);

	GLUI_Panel *text_render_optimal = new GLUI_Panel(rendering_panel_up, "Optimal Tet rendering", true);
	GLUI_RadioGroup *render_optimal = new GLUI_RadioGroup(text_render_optimal, &render_optimal_option, OPTIMAL_TET_RENDER, control_cb);

	new GLUI_RadioButton(render_optimal, "Off");
	new GLUI_RadioButton(render_optimal, "On");
	glui->add_column_to_panel(rendering_panel_up, false);

	GLUI_Panel *text_render_ground = new GLUI_Panel(rendering_panel_up, "Ground rendering", true);
	GLUI_RadioGroup *render_ground = new GLUI_RadioGroup(text_render_ground, &render_ground_option, GROUND_RENDER, control_cb);

	new GLUI_RadioButton(render_ground, "Off");
	new GLUI_RadioButton(render_ground, "On");

	check_deform[44]= new GLUI_Checkbox(rendering_panel_down, "Off", &renderDeform[44], DEFORM_TET_RENDER, control_cb); //off면 모두 다 없애버령

	new GLUI_StaticText(rendering_panel_down, "");
	check_deform[0] = new GLUI_Checkbox(rendering_panel_down, "Static Face 0", &renderDeform[0], 0, control_cb);
	check_deform[1] = new GLUI_Checkbox(rendering_panel_down, "Static Face 1", &renderDeform[1], 1, control_cb);
	check_deform[2] = new GLUI_Checkbox(rendering_panel_down, "Static Face 2", &renderDeform[2], 2, control_cb);
	check_deform[3] = new GLUI_Checkbox(rendering_panel_down, "Static Face 3", &renderDeform[3], 3, control_cb);
	new GLUI_StaticText(rendering_panel_down, "");
	check_deform[4] = new GLUI_Checkbox(rendering_panel_down, "Deform Face 0", &renderDeform[4], 4, control_cb);
	check_deform[5] = new GLUI_Checkbox(rendering_panel_down, "Deform Face 1", &renderDeform[5], 5, control_cb);
	check_deform[6] = new GLUI_Checkbox(rendering_panel_down, "Deform Face 2", &renderDeform[6], 6, control_cb);
	check_deform[7] = new GLUI_Checkbox(rendering_panel_down, "Deform Face 3", &renderDeform[7], 7, control_cb);
	glui->add_column_to_panel(rendering_panel_down);

	check_deform[8] = new GLUI_Checkbox(rendering_panel_down, "S Edge 0 D Edge 0", &renderDeform[8], 8, control_cb);
	check_deform[9] = new GLUI_Checkbox(rendering_panel_down, "S Edge 0 D Edge 1", &renderDeform[9], 9, control_cb);
	check_deform[10] = new GLUI_Checkbox(rendering_panel_down, "S Edge 0 D Edge 2", &renderDeform[10], 10, control_cb);
	check_deform[11] = new GLUI_Checkbox(rendering_panel_down, "S Edge 0 D Edge 3", &renderDeform[11], 11, control_cb);
	check_deform[12] = new GLUI_Checkbox(rendering_panel_down, "S Edge 0 D Edge 4", &renderDeform[12], 12, control_cb);
	check_deform[13] = new GLUI_Checkbox(rendering_panel_down, "S Edge 0 D Edge 5", &renderDeform[13], 13, control_cb);
	new GLUI_StaticText(rendering_panel_down, "");
	check_deform[14] = new GLUI_Checkbox(rendering_panel_down, "S Edge 1 D Edge 0", &renderDeform[14], 14, control_cb);
	check_deform[15] = new GLUI_Checkbox(rendering_panel_down, "S Edge 1 D Edge 1", &renderDeform[15], 15, control_cb);
	check_deform[16] = new GLUI_Checkbox(rendering_panel_down, "S Edge 1 D Edge 2", &renderDeform[16], 16, control_cb);
	check_deform[17] = new GLUI_Checkbox(rendering_panel_down, "S Edge 1 D Edge 3", &renderDeform[17], 17, control_cb);
	check_deform[18] = new GLUI_Checkbox(rendering_panel_down, "S Edge 1 D Edge 4", &renderDeform[18], 18, control_cb);
	check_deform[19] = new GLUI_Checkbox(rendering_panel_down, "S Edge 1 D Edge 5", &renderDeform[19], 19, control_cb);
	glui->add_column_to_panel(rendering_panel_down);

	check_deform[20] = new GLUI_Checkbox(rendering_panel_down, "S Edge 2 D Edge 0", &renderDeform[20], 20, control_cb);
	check_deform[21] = new GLUI_Checkbox(rendering_panel_down, "S Edge 2 D Edge 1", &renderDeform[21], 21, control_cb);
	check_deform[22] = new GLUI_Checkbox(rendering_panel_down, "S Edge 2 D Edge 2", &renderDeform[22], 22, control_cb);
	check_deform[23] = new GLUI_Checkbox(rendering_panel_down, "S Edge 2 D Edge 3", &renderDeform[23], 23, control_cb);
	check_deform[24] = new GLUI_Checkbox(rendering_panel_down, "S Edge 2 D Edge 4", &renderDeform[24], 24, control_cb);
	check_deform[25] = new GLUI_Checkbox(rendering_panel_down, "S Edge 2 D Edge 5", &renderDeform[25], 25, control_cb);
	new GLUI_StaticText(rendering_panel_down, "");
	check_deform[26] = new GLUI_Checkbox(rendering_panel_down, "S Edge 3 D Edge 0", &renderDeform[26], 26, control_cb);
	check_deform[27] = new GLUI_Checkbox(rendering_panel_down, "S Edge 3 D Edge 1", &renderDeform[27], 27, control_cb);
	check_deform[28] = new GLUI_Checkbox(rendering_panel_down, "S Edge 3 D Edge 2", &renderDeform[28], 28, control_cb);
	check_deform[29] = new GLUI_Checkbox(rendering_panel_down, "S Edge 3 D Edge 3", &renderDeform[29], 29, control_cb);
	check_deform[30] = new GLUI_Checkbox(rendering_panel_down, "S Edge 3 D Edge 4", &renderDeform[30], 30, control_cb);
	check_deform[31] = new GLUI_Checkbox(rendering_panel_down, "S Edge 3 D Edge 5", &renderDeform[31], 31, control_cb);
	glui->add_column_to_panel(rendering_panel_down);

	check_deform[32] = new GLUI_Checkbox(rendering_panel_down, "S Edge 4 D Edge 0", &renderDeform[32], 32, control_cb);
	check_deform[33] = new GLUI_Checkbox(rendering_panel_down, "S Edge 4 D Edge 1", &renderDeform[33], 33, control_cb);
	check_deform[34] = new GLUI_Checkbox(rendering_panel_down, "S Edge 4 D Edge 2", &renderDeform[34], 34, control_cb);
	check_deform[35] = new GLUI_Checkbox(rendering_panel_down, "S Edge 4 D Edge 3", &renderDeform[35], 35, control_cb);
	check_deform[36] = new GLUI_Checkbox(rendering_panel_down, "S Edge 4 D Edge 4", &renderDeform[36], 36, control_cb);
	check_deform[37] = new GLUI_Checkbox(rendering_panel_down, "S Edge 4 D Edge 5", &renderDeform[37], 37, control_cb);
	new GLUI_StaticText(rendering_panel_down, "");
	check_deform[38] = new GLUI_Checkbox(rendering_panel_down, "S Edge 5 D Edge 0", &renderDeform[38], 38, control_cb);
	check_deform[39] = new GLUI_Checkbox(rendering_panel_down, "S Edge 5 D Edge 1", &renderDeform[39], 39, control_cb);
	check_deform[40] = new GLUI_Checkbox(rendering_panel_down, "S Edge 5 D Edge 2", &renderDeform[40], 40, control_cb);
	check_deform[41] = new GLUI_Checkbox(rendering_panel_down, "S Edge 5 D Edge 3", &renderDeform[41], 41, control_cb);
	check_deform[42] = new GLUI_Checkbox(rendering_panel_down, "S Edge 5 D Edge 4", &renderDeform[42], 42, control_cb);
	check_deform[43] = new GLUI_Checkbox(rendering_panel_down, "S Edge 5 D Edge 5", &renderDeform[43], 43, control_cb);


	/**** Link windows to GLUI, and register idle callback ******/

	glui->set_main_gfx_window(main_window);


	/*** Create the bottom subwindow ***/
	glui2 = GLUI_Master.create_glui_subwindow(main_window,
		GLUI_SUBWINDOW_BOTTOM);
	glui2->set_main_gfx_window(main_window);

	GLUI_Rotation *view_rot = new GLUI_Rotation(glui2, "Objects", view_rotate);
	view_rot->set_spin(1.0);
	new GLUI_Column(glui2, false);
	GLUI_Rotation *lights_rot = new GLUI_Rotation(glui2, "Blue Light", lights_rotation);
	lights_rot->set_spin(.82);
	new GLUI_Column(glui2, false);
	GLUI_Translation *trans_xy =
		new GLUI_Translation(glui2, "Objects XY", GLUI_TRANSLATION_XY, obj_pos);
	trans_xy->set_speed(.005);
	new GLUI_Column(glui2, false);
	GLUI_Translation *trans_x =
		new GLUI_Translation(glui2, "Objects X", GLUI_TRANSLATION_X, obj_pos);
	trans_x->set_speed(.005);
	new GLUI_Column(glui2, false);
	GLUI_Translation *trans_y =
		new GLUI_Translation(glui2, "Objects Y", GLUI_TRANSLATION_Y, &obj_pos[1]);
	trans_y->set_speed(.005);
	new GLUI_Column(glui2, false);
	GLUI_Translation *trans_z =
		new GLUI_Translation(glui2, "Objects Z", GLUI_TRANSLATION_Z, &obj_pos[2]);
	trans_z->set_speed(.005);

#if 0
	/**** We register the idle callback with GLUI, *not* with GLUT ****/
	GLUI_Master.set_glutIdleFunc(myGlutIdle);
#endif

	/**** Regular GLUT main loop ****/
	defPD = new DefPD();
	glutMainLoop();

	return EXIT_SUCCESS;
}

