#include <bits/stdc++.h>

using namespace std;

#define EXPAND_DISTANCE 0.5
#define GOAL_SAMPLE_RATE 20
#define MAX_INTER 5000
#define MIN_RAND -2
#define MAX_RAND 15

// node (x, y, parent)
struct Node
{
    float x, y;
    float cost;
    int parent;

    Node(float x, float y, int idx)
    {
        this->x = x;
        this->y = y;
        this->cost = 0.0;
        this->parent = idx;
    }
};

typedef pair<float, float> PosPair;

class RRT
{
private:

public:
    Node* start;
    Node* goal;
    vector<Node*> node_list;
    vector<PosPair> path;
    int obst_num;

    RRT(Node* s, Node* g, int n){ start = s; goal = g; obst_num = n;}

    static int nearest_node_list(vector<Node*> node_list, PosPair rnd)
    {
        int min_idx = -1;

        double min_dist, dist;

        for (int i = 0; i < node_list.size(); ++i) {
            dist = (node_list[i]->x - rnd.first) * (node_list[i]->x - rnd.first) +
                   (node_list[i]->y - rnd.second) * (node_list[i]->y - rnd.second);
            if (i == 0) {min_dist = dist; min_idx = i;}
            else
            {
                if (dist < min_dist)
                {
                    min_dist = dist;
                    min_idx = i;
                }
            }
        }

        return min_idx;
    }

    Node* steer(PosPair rnd, int nrst_idx)
    {
        Node* nrst_node;
        float angle;

        nrst_node = node_list[nrst_idx];

        angle = atan2(rnd.second - nrst_node->y, rnd.first - nrst_node->x);

        Node* new_node = new Node(nrst_node->x, nrst_node->y, nrst_idx);
        new_node->x += EXPAND_DISTANCE * cos(angle);
        new_node->y += EXPAND_DISTANCE * sin(angle);

        new_node->cost += EXPAND_DISTANCE;

        new_node->parent = nrst_idx;

        return new_node;
    }

    bool check_collision(Node* node, int obstacles[][3])
    {
        float dx, dy, d;

        for (int i = 0; i < obst_num; ++i) {
            dx = obstacles[i][0] - node->x;
            dy = obstacles[i][1] - node->y;
            d = sqrt(dx * dx + dy * dy);

            if (d <= obstacles[i][2]) {return false;} // collision
        }

        return true; // safe
    }

    vector<int> find_near_nodes(float new_x, float new_y)
    {
        vector<int> near_idxes;
        vector<float> dist_list;
        float dist;

        float nodes_num = node_list.size();

        // 2 dimension
        float radius = 50.0 * sqrt(log(nodes_num) / nodes_num);

        // find
        for (int i = 0; i < node_list.size(); ++i) {
            dist = (node_list[i]->x - new_x) * (node_list[i]->x - new_x) +
                    (node_list[i]->y - new_y) * (node_list[i]->y - new_y);
            dist_list.push_back(dist);
        }

        for (int j = 0; j < dist_list.size(); ++j) {
            if (dist_list[j] <= (radius * radius))
            {
                near_idxes.push_back(j);
            }
        }

        return near_idxes;
    }

    bool check_collision_extend(Node* near_node, float theta, float d, int obstacles[][3])
    {
        Node* tmp_node = new Node(near_node->x, near_node->y, near_node->parent);

        for (int i = 0; i < int(d / EXPAND_DISTANCE); ++i) {
            tmp_node->x += EXPAND_DISTANCE * cos(theta);
            tmp_node->y += EXPAND_DISTANCE * sin(theta);
            if (!check_collision(tmp_node, obstacles))
            {
                return false;
            }
        }

        return true;
    }

    Node* choose_parent(Node* new_node, vector<int> near_idxes, int obstacles[][3])
    {
        if (near_idxes.empty()) {return new_node;}

        vector<float> dlist;
        float dx, dy, d, theta;
        float min_cost;
        int min_idx;

        for (auto ni : near_idxes) {
            dx = new_node->x - node_list[ni]->x;
            dy = new_node->y - node_list[ni]->y;
            d = sqrt(dx * dx - dy * dy);
            theta = atan2(dy, dx);
            if (check_collision_extend(node_list[ni], theta, d, obstacles))
            {
                dlist.push_back(node_list[ni]->cost + d);
            }
            else
            {
                dlist.push_back(float(INFINITY));
            }
        }

        min_cost = *min_element(dlist.begin(), dlist.end());
        min_idx = min_element(dlist.begin(), dlist.end()) - dlist.begin();

        if (min_cost == float(INFINITY))
        {
            cout << "Min cost is inf" << endl;
            return new_node;
        }

        new_node->cost = min_cost;
        new_node->parent = min_idx;

        return new_node;
    }

    void rewire(Node* new_node, vector<int> near_idxes, int obstacles[][3])
    {
        Node* near_node;
        float dx, dy, d, s_cost, theta;

        int n_node = near_idxes.size();

        for (auto ni : near_idxes) {
            near_node = node_list[ni];

            dx = new_node->x - near_node->x;
            dy = new_node->y - near_node->y;
            d = sqrt(dx * dx + dy * dy);

            s_cost = new_node->cost + d;

            if (near_node->cost > s_cost)
            {
                theta = atan2(dy, dx);
                if (check_collision_extend(near_node, theta, d, obstacles))
                {
                    near_node->parent = n_node - 1;
                    near_node->cost = s_cost;
                }
            }
        }
    }

    void planning(int obstacles[][3])
    {
        // add start node
        node_list.push_back(start);

        // random function
        mt19937 mt{random_device{}()};
        uniform_int_distribution<int> dist_int(0, 100);
        uniform_real_distribution<float> dist_float(MIN_RAND, MAX_RAND);

        // sampled node
        PosPair rnd;

        // nearest node's data
        int nrst_idx;

        // near node's index list
        vector<int> near_idxes;

        // check goal
        float dx, dy, d;
        int last_idx;
        Node* last_node;

        while (true)
        {
            // random sampling
            if (dist_int(mt) > GOAL_SAMPLE_RATE)
            {
                rnd = make_pair(dist_float(mt), dist_float(mt));
            }
            else
            {
                rnd = make_pair(goal->x, goal->y);
            }

            // find nearest node
            nrst_idx = nearest_node_list(node_list, rnd);

            // steer
            Node* new_node = steer(rnd, nrst_idx);

            if (!check_collision(new_node, obstacles)) {continue;}

            // find near nodes
            near_idxes = find_near_nodes(new_node->x, new_node->y);

            // choose parent
            new_node = choose_parent(new_node, near_idxes, obstacles);
            node_list.push_back(new_node);

            // rewire
            rewire(new_node, near_idxes, obstacles);

            // check goal
//            dx = new_node->x - goal->x;
//            dy = new_node->y - goal->y;
//            d = sqrt(dx * dx + dy * dy);
//            if (d <= EXPAND_DISTANCE)
//            {
//                cout << "Goal!!" << endl;
//                break;
//            }
        }

        // path
//        path.push_back(make_pair(goal->x, goal->y));
//        last_idx = node_list.size() - 1;
//        while (node_list[last_idx]->parent != -1)
//        {
//            last_node = node_list[last_idx];
//            path.push_back(make_pair(last_node->x, last_node->y));
//            last_idx = last_node->parent;
//        }
//        path.push_back(make_pair(start->x, start->y));
//
//        // print
//        reverse(path.begin(), path.end());
//        for (int i = 0; i < path.size() ; ++i) {
//            if (i == path.size() - 1)
//            {
//                cout << "(" << path[i].first << " " << path[i].second << ")" << endl;
//            }
//            else
//            {
//                cout << "(" << path[i].first << " " << path[i].second << ")" << "->";
//            }
//        }
    }
};

int main() {
    // obstacles (x, y, size)
    int obstacles[7][3] = {
            {5, 5, 2},
            {3, 6, 2},
            {3, 8, 2},
            {3, 10, 2},
            {7, 5, 2},
            {7, 9, 2},
            {9, 5, 2}
    };

    int size = sizeof(obstacles) / sizeof(*obstacles);

    // start position
    Node* start = new Node(0, 0, -1);

    // goal position
    Node* goal = new Node(6, 7, -1);

    RRT rrt = RRT(start, goal, size);

    rrt.planning(obstacles);

    return 0;
}
