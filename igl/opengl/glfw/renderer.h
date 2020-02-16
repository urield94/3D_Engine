#pragma once
#include <igl/igl_inline.h>
#include <vector>
#include <functional>
#include <igl/opengl/ViewerCore.h>
#include <igl/opengl/glfw/Viewer.h>
#include <tutorial/sandBox/DB.h>
#include "igl/AABB.h"
#include <tutorial/sandBox/DB.h>

struct GLFWwindow;

class Renderer
{
public:
	Renderer();
	~Renderer();
	IGL_INLINE void Draw(GLFWwindow *window);
	IGL_INLINE void Init(igl::opengl::glfw::Viewer *viewer, int player_score, int player_level, int player_id, DB db);

	//IGL_INLINE bool key_pressed(unsigned int unicode_key, int modifiers);

	// Returns **true** if action should be cancelled.
	std::function<bool(GLFWwindow* window)> callback_init;
	std::function<bool(GLFWwindow* window)> callback_pre_draw;
	std::function<bool(GLFWwindow* window)> callback_post_draw;
	std::function<bool(GLFWwindow* window, int button, int modifier)> callback_mouse_down;
	std::function<bool(GLFWwindow* window, int button, int modifier)> callback_mouse_up;
	std::function<bool(GLFWwindow* window, int mouse_x, int mouse_y)> callback_mouse_move;
	std::function<bool(GLFWwindow* window, float delta_y)> callback_mouse_scroll;
	std::function<bool(GLFWwindow* window, unsigned int key, int modifiers)> callback_key_pressed;
	std::function<bool(GLFWwindow* window, int w, int h)> callback_post_resize;
	// THESE SHOULD BE DEPRECATED:
	std::function<bool(GLFWwindow* window, unsigned int key, int modifiers)> callback_key_down;
	std::function<bool(GLFWwindow* window, unsigned int key, int modifiers)> callback_key_up;
	// Pointers to per-callback data
	void* callback_init_data;
	void* callback_pre_draw_data;
	void* callback_post_draw_data;
	void* callback_mouse_down_data;
	void* callback_mouse_up_data;
	void* callback_mouse_move_data;
	void* callback_mouse_scroll_data;
	void* callback_key_pressed_data;
	void* callback_key_down_data;
	void* callback_key_up_data;


	////////////////////////////
	// Multi-viewport methods //
	////////////////////////////

	// Return the current viewport, or the viewport corresponding to a given unique identifier
	//
	// Inputs:
	//   core_id  unique identifier corresponding to the desired viewport (current viewport if 0)
	IGL_INLINE igl::opengl::ViewerCore& core(unsigned core_id = 0);
	IGL_INLINE const igl::opengl::ViewerCore& core(unsigned core_id = 0) const;

	// Append a new "slot" for a viewport (i.e., copy properties of the current viewport, only
	// changing the viewport size/position)
	//
	// Inputs:
	//   viewport      Vector specifying the viewport origin and size in screen coordinates.
	//   append_empty  If true, existing meshes are hidden on the new viewport.
	//
	// Returns the unique id of the newly inserted viewport. There can be a maximum of 31
	//   viewports created in the same viewport. Erasing a viewport does not change the id of
	//   other existing viewports
	IGL_INLINE int append_core(Eigen::Vector4f viewport, bool append_empty = false);

	// Erase a viewport
	//
	// Inputs:
	//   index  index of the viewport to erase
	IGL_INLINE bool erase_core(const size_t index);

	// Retrieve viewport index from its unique identifier
	// Returns 0 if not found
	IGL_INLINE size_t core_index(const int id) const;

	// Change selected_core_index to the viewport containing the mouse
	// (current_mouse_x, current_mouse_y)
	// IGL_INLINE void select_hovered_core();

	// Callbacks
	int Picking(double x, double y);
	IGL_INLINE bool key_pressed(unsigned int unicode_key, int modifier);
	IGL_INLINE void resize(GLFWwindow* window,int w, int h); // explicitly set window size
	IGL_INLINE void post_resize(GLFWwindow* window, int w, int h); // external resize due to user interaction
	void SetScene(igl::opengl::glfw::Viewer* scn);
	void UpdatePosition(double xpos, double ypos);
	void MouseProcessing(int button);
	inline igl::opengl::glfw::Viewer* GetScene() {
		return scn;
	}
	inline void ChangeCamera(int unicode_key)
	{
		selected_core_index =
				(selected_core_index + core_list.size() + (unicode_key == ']' ? 1 : -1)) % core_list.size();

	}
	void InitSystem();
	void draw_axis(igl::opengl::ViewerData & mesh);
	void ResizeByScrolling(double x, double y);
	void IK_Solver();
	bool object_picked = false;

	Eigen::Matrix4f GetAncestorTrans(int link_index);
	Eigen::Matrix4f GetAncestorTransIfNeeded(int index){
		return ((index <= 0 || index >= scn->links_number) ? Eigen::Matrix4f::Identity() : GetAncestorTrans(index));
	};

	Eigen::Matrix3f GetAncestorInverse(int link_index);
	Eigen::Matrix3f GetAncestorInverseIfNeeded(int index){
		return ((index <= 0 || index >= scn->links_number) ? Eigen::Matrix3f::Identity() : GetAncestorInverse(index));
	};

	Eigen::Matrix3f GetAncestorRotation(int link_index);

	bool IsBoxesColide(igl::opengl::ViewerData &obj1, igl::opengl::ViewerData &obj2,
                       igl::AABB<Eigen::MatrixXd, 3> tree1, igl::AABB<Eigen::MatrixXd, 3> tree2);
	std::vector<igl::AABB<Eigen::MatrixXd,3>> trees;
	std::vector<std::string> object_paths;
	igl::opengl::glfw::Viewer* scn;
	// Stores all the viewing options
	std::vector<igl::opengl::ViewerCore> core_list;
	int level_score = 0;
	int final_score = 0;
	int level = 0;
	void SetVelocity(igl::opengl::ViewerData &obj);
	void ResetObject(igl::opengl::ViewerData &obj);
    int game_id;
    DB db;
	void SetBackground();
	void ResetLevel();
private:
	size_t selected_core_index;
	int next_core_id;
	float highdpi;
	double xold, yold, xrel, yrel;
	bool prerotation = true;
    typedef struct OBBSatVars{
        Eigen::Vector3f A0; Eigen::Vector3f A1; Eigen::Vector3f A2;
        Eigen::Vector3f B0; Eigen::Vector3f B1; Eigen::Vector3f B2;
        float a0; float a1; float a2; float b0; float b1; float b2;
        float c00; float c01; float c02;
        float c10; float c11; float c12;
        float c20; float c21; float c22;
        Eigen::Vector3f D;
    }OBBSatVars;
    bool OBBCheckSat(OBBSatVars vars);

};
