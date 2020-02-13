

#include <chrono>
#include <thread>

#include "../gl.h"
#include "Display.h"

#include "igl/igl_inline.h"
#include <igl/get_seconds.h>


static void glfw_error_callback(int error, const char* description)
{
	fputs(description, stderr);
}

Display::Display(int windowWidth, int windowHeight, const std::string& title)
{
	bool resizable = true, fullscreen = false;
		glfwSetErrorCallback(glfw_error_callback);
		if (!glfwInit())
		{
			exit(EXIT_FAILURE);
		}
		glfwWindowHint(GLFW_SAMPLES, 8);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		
//#ifdef __APPLE__
//		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
//#endif
//		if (fullscreen)
//		{
//			GLFWmonitor* monitor = glfwGetPrimaryMonitor();
//			const GLFWvidmode* mode = glfwGetVideoMode(monitor);
//			window = glfwCreateWindow(mode->width, mode->height, title.c_str(), monitor, nullptr);
//			windowWidth = mode->width;
//			windowHeight = mode->height;
//		}
//		else
//		{
			// Set default windows width
			//if (windowWidth <= 0 & core_list.size() == 1 && renderer->core().viewport[2] > 0)
			//	windowWidth = renderer->core().viewport[2];
			//else 
			//	if (windowWidth <= 0)
			//	windowWidth = 1280;
			//// Set default windows height
			//if (windowHeight <= 0 & core_list.size() == 1 && renderer->core().viewport[3] > 0)
			//	windowHeight = renderer->core().viewport[3];
			//else if (windowHeight <= 0)
			//	windowHeight = 800;
//			window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), nullptr, nullptr);
//		}
		window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), nullptr, nullptr);
		if (!window)
		{
			glfwTerminate();
			exit(EXIT_FAILURE);
		}
		glfwMakeContextCurrent(window);
		// Load OpenGL and its extensions
		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		{
			printf("Failed to load OpenGL and its extensions\n");
			exit(EXIT_FAILURE);
		}
#if defined(DEBUG) || defined(_DEBUG)
		printf("OpenGL Version %d.%d loaded\n", GLVersion.major, GLVersion.minor);
		int major, minor, rev;
		major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
		minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
		rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
		printf("OpenGL version received: %d.%d.%d\n", major, minor, rev);
		printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
		printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));
#endif
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		//Tamir: changes from here
		// Initialize FormScreen
	   // __viewer = this;
		// Register callbacks
		//glfwSetKeyCallback(window, glfw_key_callback);
		//glfwSetCursorPosCallback(window,glfw_mouse_move);
		//glfwSetScrollCallback(window, glfw_mouse_scroll);
		//glfwSetMouseButtonCallback(window, glfw_mouse_press);
		//glfwSetWindowSizeCallback(window,glfw_window_size);
	
		
		//glfwSetCharModsCallback(window,glfw_char_mods_callback);
		//glfwSetDropCallback(window,glfw_drop_callback);
		// Handle retina displays (windows and mac)
		//int width, height;
		//glfwGetFramebufferSize(window, &width, &height);
		//int width_window, height_window;
		//glfwGetWindowSize(window, &width_window, &height_window);
		//highdpi = windowWidth/width_window;
		
		//glfw_window_size(window,width_window,height_window);
		//opengl.init();
//		core().align_camera_center(data().V, data().F);
		// Initialize IGL viewer
//		init();
		
}


void Display::load_objects(Renderer *rndr) {
	int i = 0;
	int group = 0;
	int old_selected = rndr->scn->selected_data_index;
    rndr->object_picked = false;
	for(auto& path: rndr->object_paths) {
		for (int j = 0; j < 9; j++) {
			rndr->scn->load_mesh_from_file(path);
			float angle = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			rndr->scn->data().MyTranslate(Eigen::Vector3f(-(rand() % (3)), (rand() % (3)), (rand() % (3))));
			if (i < rndr->scn->links_number + 4) {
				rndr->scn->data().MyRotate(Eigen::Vector3f(-(rand() % (3)),
														   (rand() % (3)),
														   -(rand() % (3))), angle);
			} else if (i < rndr->scn->links_number + 8) {
				rndr->scn->data().MyRotate(Eigen::Vector3f((rand() % (3)),
														   -(rand() % (3)),
														   (rand() % (3))), -angle);
			} else if (i < rndr->scn->links_number + 12) {
				rndr->scn->data().MyRotate(Eigen::Vector3f((rand() % (3)),
														   (rand() % (3)),
														   -(rand() % (3))), -angle);
			} else {
				rndr->scn->data().MyRotate(Eigen::Vector3f((rand() % (3)),
														   (rand() % (3)),
														   (rand() % (3))), angle);
			}

			rndr->trees.resize(rndr->scn->data_list.size());
			igl::AABB<Eigen::MatrixXd, 3> tree;
			tree.init(rndr->scn->data().V,
					  rndr->scn->data().F);
			rndr->trees[rndr->scn->mesh_index(rndr->scn->data().id)] = tree;

			rndr->scn->data().set_visible(true, rndr->core().id);
			rndr->scn->data().copy_options(rndr->core_list[0], rndr->core());

			rndr->scn->data().score_group = group;

			rndr->ResetObject(rndr->scn->data());

			Eigen::MatrixXd C(1,3);
			C << (rand() % (5)), (rand() % (5)), (rand() % (5));
			rndr->scn->data().set_colors(C);

			i++;

		}
		group++;
	}
    rndr->scn->selected_data_index = old_selected;
    rndr->object_picked = true;
}

bool Display::launch_rendering(bool loop)
{
	// glfwMakeContextCurrent(window);
	// Rendering loop
	const int num_extra_frames = 5;
	int frame_counter = 0;
	int windowWidth, windowHeight;
	//main loop
	Renderer* renderer = (Renderer*)glfwGetWindowUserPointer(window);
	glfwGetWindowSize(window, &windowWidth, &windowHeight);
	renderer->post_resize(window, windowWidth, windowHeight);

	load_objects(renderer);

	double last_first_time = igl::get_seconds();

	std::cout << "\n----LEVEL " << renderer->level << "----" << std::endl;
	while (!glfwWindowShouldClose(window))
	{

		double tic = igl::get_seconds();
		if (renderer->object_picked) {
			renderer->IK_Solver();
		}
		renderer->draw(window);
		glfwSwapBuffers(window);
		if (renderer->core().is_animating || frame_counter++ < num_extra_frames)
		{//motion
			glfwPollEvents();
			// In microseconds
			double duration = 1000000. * (igl::get_seconds() - tic);
			const double min_duration = 1000000. / renderer->core().animation_max_fps;
			if (duration < min_duration)
			{
				std::this_thread::sleep_for(std::chrono::microseconds((int)(min_duration - duration)));
			}
		}
		else
		{
			glfwPollEvents();
			frame_counter = 0;
		}

		if(renderer->level_score >= 50){
			renderer->level++;
			renderer->db.SetLevel(renderer->game_id,renderer->final_score, renderer->level);

			char ans = 'N';
			std::cout << "Final level level_score: " << renderer->level_score << std::endl;
			std::cout << "Do you want to continue to the next level? (Y/N): ";
			std::cin >> ans;
			if(ans == 'N' || ans == 'n'){
				std::cout << "\n*******************\n* FINAL SCORE: " << renderer->final_score << " *\n*******************" << std::endl;
				return EXIT_SUCCESS;
			}

			renderer->level_score = 0;
			renderer->SetBackground();
			std::cout << "---------------\n" << std::endl;
			std::cout << "----LEVEL " << renderer->level << "----" << std::endl;
		}
		if( (renderer->level > 10) && (tic - last_first_time > 100 + renderer->level)){
			load_objects(renderer);
			last_first_time = igl::get_seconds();
		}
		if (!loop)
			return !glfwWindowShouldClose(window);

#ifdef __APPLE__
		static bool first_time_hack = true;
		if (first_time_hack) {
			glfwHideWindow(window);
			glfwShowWindow(window);
			first_time_hack = false;
		}
#endif
	}
	return EXIT_SUCCESS;
}

void Display::AddKeyCallBack(void(*keyCallback)(GLFWwindow*, int, int, int, int))
{
	glfwSetKeyCallback(window, (void(*)(GLFWwindow*, int, int, int, int))keyCallback);//{

}

void Display::AddMouseCallBacks(void (*mousebuttonfun)(GLFWwindow*, int, int, int), void (*scrollfun)(GLFWwindow*, double, double), void (*cursorposfun)(GLFWwindow*, double, double))
{
	glfwSetMouseButtonCallback(window, mousebuttonfun);
	glfwSetScrollCallback(window, scrollfun);
	glfwSetCursorPosCallback(window, cursorposfun);
}

void Display::AddResizeCallBack(void (*windowsizefun)(GLFWwindow*, int, int))
{
	glfwSetWindowSizeCallback(window, windowsizefun);
}

void Display::SetRenderer(void* userPointer)
{
	
	glfwSetWindowUserPointer(window, userPointer);
	
}

void* Display::GetScene()
{
	return glfwGetWindowUserPointer(window);
}

void Display::SwapBuffers()
{
	glfwSwapBuffers(window);
}

void Display::PollEvents()
{
	glfwPollEvents();
}

Display::~Display()
{
	glfwDestroyWindow(window);
	glfwTerminate();
}

