#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

/*
 * A*探索アルゴリズムによる2次元グリッドの最短経路探索
 * 
 * - 'S' がスタート、'G' がゴール、'#' が障害物
 * - マンハッタン距離をヒューリスティックに使用
 * - 経路を見つけたら座標を順に出力
 */

struct Node {
    int x, y;
    int g, h; // g: スタートからの距離, h: ヒューリスティック値
    Node* parent;

    int f() const { return g + h; }
};

// 優先度付きキューでf値が小さい順に並べる
struct Compare {
    bool operator()(Node* a, Node* b) {
        return a->f() > b->f();
    }
};

int heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2); // マンハッタン距離
}

bool is_valid(int x, int y, const std::vector<std::string>& grid) {
    return x >= 0 && y >= 0 && x < (int)grid.size() && y < (int)grid[0].size() && grid[x][y] != '#';
}

void reconstruct_path(Node* goal) {
    std::vector<std::pair<int,int>> path;
    for (Node* n = goal; n != nullptr; n = n->parent) {
        path.push_back({n->x, n->y});
    }
    std::reverse(path.begin(), path.end());

    std::cout << "Path found:\n";
    for (auto& p : path) {
        std::cout << "(" << p.first << ", " << p.second << ")\n";
    }
}

void astar(std::vector<std::string>& grid) {
    int startX, startY, goalX, goalY;

    // スタートとゴールの位置を探す
    for (int i = 0; i < (int)grid.size(); i++) {
        for (int j = 0; j < (int)grid[0].size(); j++) {
            if (grid[i][j] == 'S') { startX = i; startY = j; }
            if (grid[i][j] == 'G') { goalX = i; goalY = j; }
        }
    }

    std::priority_queue<Node*, std::vector<Node*>, Compare> open;
    std::vector<std::vector<bool>> closed(grid.size(), std::vector<bool>(grid[0].size(), false));

    Node* start = new Node{startX, startY, 0, heuristic(startX, startY, goalX, goalY), nullptr};
    open.push(start);

    int dx[4] = {-1, 1, 0, 0};
    int dy[4] = {0, 0, -1, 1};

    while (!open.empty()) {
        Node* current = open.top();
        open.pop();

        if (current->x == goalX && current->y == goalY) {
            reconstruct_path(current);
            return;
        }

        closed[current->x][current->y] = true;

        for (int i = 0; i < 4; i++) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];

            if (!is_valid(nx, ny, grid) || closed[nx][ny]) continue;

            int g_cost = current->g + 1;
            int h_cost = heuristic(nx, ny, goalX, goalY);

            Node* neighbor = new Node{nx, ny, g_cost, h_cost, current};
            open.push(neighbor);
        }
    }

    std::cout << "No path found.\n";
}

int main() {
    std::vector<std::string> grid = {
        "S....",
        ".##..",
        "..#..",
        "..##.",
        "...G."
    };

    astar(grid);

    return 0;
}
