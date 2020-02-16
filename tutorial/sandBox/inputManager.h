#pragma once
#include "igl/opengl/glfw/Display.h"
#include <igl/look_at.h>

static void change_color(igl::opengl::glfw::Viewer* scn, int color){ // color = 0 for yellow 1 for red
	Eigen::MatrixXd C(1,3);
	if((scn->selected_data_index != -1 || scn->selected_data_index == scn->links_number) && !color){
		if(scn->selected_data_index < scn->links_number)
			C << 1,1,0;
		else
			C << (rand() % (5)), (rand() % (5)), (rand() % (5));
		scn->data().set_colors(C);
	}
	if((scn->selected_data_index != -1 || scn->selected_data_index == scn->links_number) && color){
		C << 1,0,0;
		scn->data().set_colors(C);
	}
}


static void glfw_mouse_press(GLFWwindow* window, int button, int action, int modifier)
{

	Renderer* rndr = (Renderer*) glfwGetWindowUserPointer(window);

	if (action == GLFW_PRESS)
	{
		double x2, y2;
		glfwGetCursorPos(window, &x2, &y2);
		igl::opengl::glfw::Viewer* scn = rndr->GetScene();
		int closest_index = 0;

		closest_index = rndr->Picking(x2, y2);

		if(closest_index == -1 || closest_index == scn->links_number)
		{
			change_color(scn, 0);
			scn->selected_data_index = -1;
		}
		else if(closest_index > scn->links_number - 1){
            change_color(scn, 0);
            scn->selected_data_index = closest_index;
            change_color(scn, 1);
            rndr->object_picked = true;
		}
		else {
			change_color(scn, 0);
			scn->selected_data_index = closest_index;
			change_color(scn, 1);
		}
		rndr->UpdatePosition(x2, y2);

	}
}


void glfw_mouse_move(GLFWwindow* window, double x, double y)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	rndr->UpdatePosition(x, y);
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
	{
		rndr->MouseProcessing(GLFW_MOUSE_BUTTON_RIGHT);
	}
	else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
	{
		rndr->MouseProcessing(GLFW_MOUSE_BUTTON_LEFT);
	}
}

static void glfw_mouse_scroll(GLFWwindow* window, double x, double y)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	rndr->ResizeByScrolling(x, y);
}

void glfw_window_size(GLFWwindow* window, int width, int height)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	//igl::opengl::glfw::Viewer* scn = rndr->GetScene();

	rndr->post_resize(window,width, height);

}

static void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int modifier)
{
	Renderer* rndr = (Renderer*) glfwGetWindowUserPointer(window);
	igl::opengl::glfw::Viewer* scn = rndr->GetScene();
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	else if(action == GLFW_PRESS || action == GLFW_REPEAT)
		switch (key)
		{
			case 'A':
			case 'a':
			{
				rndr->core().is_animating = !rndr->core().is_animating;
				break;
			}
			case 'F':
			case 'f':
			{
				scn->data().set_face_based(!scn->data().face_based);
				break;
			}
			case 'I':
			case 'i':
			{
				scn->data().dirty |= igl::opengl::MeshGL::DIRTY_NORMAL;
				scn->data().invert_normals = !scn->data().invert_normals;
				break;
			}
			case 'L':
			case 'l':
			{
				rndr->core().toggle(scn->data().show_lines);
				break;
			}
			case 'O':
			case 'o':
			{
				rndr->core().orthographic = !rndr->core().orthographic;
				break;
			}
			case 'T':
			case 't':
			{
				rndr->core().toggle(scn->data().show_faces);
				break;
			}
			case '1':
			case '2':
			{
				scn->selected_data_index =
						(scn->selected_data_index + scn->data_list.size() + (key == '2' ? 1 : -1)) % scn->data_list.size();
				break;
			}
			case '[':
			case ']':
			{
				rndr->ChangeCamera(key);
				break;
			}
			case ';':
				scn->data().show_vertid = !scn->data().show_vertid;
				break;
			case ':':
				scn->data().show_faceid = !scn->data().show_faceid;
				break;
			case ' ':
				rndr->object_picked = !rndr->object_picked;
				break;
			case GLFW_KEY_LEFT://(left arrow)
				std::cout << "Left rotation" << std::endl;
				scn->data().MyRotate(Eigen::Vector3f(0,-1,0),0.1);
				break;
			case GLFW_KEY_UP://(up arrow)
				std::cout << "up rotation" << std::endl;
				scn->data().MyRotate(Eigen::Vector3f(-1,0,0),0.1);
				break;
			case GLFW_KEY_RIGHT://(right arrow)
				std::cout << "right rotation" << std::endl;
				scn->data().MyRotate(Eigen::Vector3f(0,1,0),0.1);
				break;
			case GLFW_KEY_DOWN ://(down arrow)
				std::cout << "down rotation" << std::endl;
				scn->data().MyRotate(Eigen::Vector3f(1,0,0),0.1);
				break;
			default: break;//do nothing
		}
}


void Init(Display& display)
{
	display.AddKeyCallBack(glfw_key_callback);
	display.AddMouseCallBacks(glfw_mouse_press, glfw_mouse_scroll, glfw_mouse_move);
	display.AddResizeCallBack(glfw_window_size);
}


