
// Dijkstra Pathfinding Visualizer using C++, OpenGL, GLFW, GLEW, and ImGui 1.60


#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <tuple>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw_gl3.h"

static const int DEFAULT_GRID_SIZE = 20;
static const int WINDOW_SIZE = 600;

enum CellState { EMPTY, OBSTACLE, START, END, VISITED, PATH };

enum PlacementMode { MODE_OBSTACLE = 0, MODE_START, MODE_END, MODE_ERASE };

int main() {
  
    if (!glfwInit()) return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    GLFWwindow* window = glfwCreateWindow(800, 640, "Dijkstra Visualizer", NULL, NULL);
    if (!window) { glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);
    glewInit();

    
    ImGui::CreateContext();
    ImGui_ImplGlfwGL3_Init(window, true);
    ImGui::StyleColorsDark();

    
    int gridSize = DEFAULT_GRID_SIZE;
    std::vector<CellState> grid;
    grid.assign(gridSize * gridSize, EMPTY);
    int startIdx = -1, endIdx = -1;

    bool runDijkstra = false;
    PlacementMode mode = MODE_OBSTACLE;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ImGui_ImplGlfwGL3_NewFrame();

        
        ImGui::SetNextWindowPos({ 0, 0 });
        ImGui::SetNextWindowSize({ 200, 640 });
        ImGui::Begin("Controls", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
        ImGui::Text("Placement Mode:");
        ImGui::RadioButton("Obstacle", (int*)&mode, MODE_OBSTACLE);
        ImGui::RadioButton("Start", (int*)&mode, MODE_START);
        ImGui::RadioButton("End", (int*)&mode, MODE_END);
        ImGui::RadioButton("Erase", (int*)&mode, MODE_ERASE);

        ImGui::Separator();
        ImGui::SliderInt("Grid Size", &gridSize, 5, 50);
        if (ImGui::Button("Reset Grid")) {
            grid.assign(gridSize * gridSize, EMPTY);
            startIdx = endIdx = -1;
        }
        if (ImGui::Button("Run Dijkstra")) {
            runDijkstra = true;
        }
        ImGui::End();

        
        static int prevSize = DEFAULT_GRID_SIZE;
        if (prevSize != gridSize) {
            grid.assign(gridSize * gridSize, EMPTY);
            startIdx = endIdx = -1;
            prevSize = gridSize;
        }

        
        if (runDijkstra && startIdx >= 0 && endIdx >= 0) {
            int N = gridSize;
            const int INF = std::numeric_limits<int>::max();
            std::vector<int> dist(N * N, INF), prev(N * N, -1);
            using Node = std::pair<int, int>;
            std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
            dist[startIdx] = 0;
            pq.push({ 0, startIdx });

            auto neighbors = [&](int idx) {
                std::vector<int> out;
                int r = idx / N, c = idx % N;
                const int dr[4] = { -1,1,0,0 }, dc[4] = { 0,0,-1,1 };
                for (int i = 0; i < 4; i++) {
                    int nr = r + dr[i], nc = c + dc[i];
                    if (nr >= 0 && nr < N && nc >= 0 && nc < N) out.push_back(nr * N + nc);
                }
                return out;
                };

            
            for (auto& cs : grid) if (cs == VISITED || cs == PATH) cs = EMPTY;

            while (!pq.empty()) {
                auto [d, u] = pq.top(); pq.pop();
                if (d > dist[u]) continue;
                if (u == endIdx) break;
                for (int v : neighbors(u)) {
                    if (grid[v] == OBSTACLE || grid[v] == START) continue;
                    int nd = d + 1;
                    if (nd < dist[v]) {
                        dist[v] = nd;
                        prev[v] = u;
                        pq.push({ nd,v });
                        if (v != endIdx) grid[v] = VISITED;
                    }
                }
            }
           
            int cur = endIdx;
            while (cur >= 0 && prev[cur] != -1) {
                cur = prev[cur];
                if (cur != startIdx) grid[cur] = PATH;
            }
            runDijkstra = false;
        }

    
        ImGui::SetNextWindowPos({ 200, 0 });
        ImGui::SetNextWindowSize({ WINDOW_SIZE, WINDOW_SIZE });
        ImGui::Begin("Grid", nullptr,
            ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove);
        ImGui::InvisibleButton("canvas", ImGui::GetWindowSize());
        ImVec2 p0 = ImGui::GetItemRectMin();
        ImVec2 p1 = ImGui::GetItemRectMax();
        ImDrawList* draw = ImGui::GetWindowDrawList();
        float cell = (p1.x - p0.x) / gridSize;

       
        if (ImGui::IsMouseClicked(0)) {
            ImVec2 m = ImGui::GetMousePos();
            if (m.x >= p0.x && m.x < p1.x && m.y >= p0.y && m.y < p1.y) {
                int c = (int)((m.x - p0.x) / cell);
                int r = (int)((m.y - p0.y) / cell);
                int idx = r * gridSize + c;
                switch (mode) {
                case MODE_OBSTACLE:
                    if (idx != startIdx && idx != endIdx) grid[idx] = (grid[idx] == OBSTACLE ? EMPTY : OBSTACLE);
                    break;
                case MODE_START:
                    if (idx != endIdx) {
                        if (startIdx >= 0) grid[startIdx] = EMPTY;
                        startIdx = idx;
                        grid[idx] = START;
                    }
                    break;
                case MODE_END:
                    if (idx != startIdx) {
                        if (endIdx >= 0) grid[endIdx] = EMPTY;
                        endIdx = idx;
                        grid[idx] = END;
                    }
                    break;
                case MODE_ERASE:
                    if (idx == startIdx) startIdx = -1;
                    if (idx == endIdx) endIdx = -1;
                    grid[idx] = EMPTY;
                    break;
                }
            }
        }

        
        for (int r = 0; r < gridSize; r++) {
            for (int c = 0; c < gridSize; c++) {
                int idx = r * gridSize + c;
                ImVec2 a = { p0.x + c * cell, p0.y + r * cell };
                ImVec2 b = { a.x + cell, a.y + cell };
                ImU32 col;
                switch (grid[idx]) {
                case EMPTY:    col = IM_COL32(255, 255, 255, 255); break;
                case OBSTACLE: col = IM_COL32(0, 0, 0, 255); break;
                case START:    col = IM_COL32(0, 255, 0, 255); break;
                case END:      col = IM_COL32(255, 0, 0, 255); break;
                case VISITED:  col = IM_COL32(0, 150, 255, 255); break;
                case PATH:     col = IM_COL32(255, 255, 0, 255); break;
                }
                draw->AddRectFilled(a, b, col);
                draw->AddRect(a, b, IM_COL32(100, 100, 100, 255));
            }
        }
        ImGui::End();

      
        ImGui::Render();
        ImGui_ImplGlfwGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

   
    ImGui_ImplGlfwGL3_Shutdown();
    ImGui::DestroyContext();
    glfwTerminate();
    return 0;
}
