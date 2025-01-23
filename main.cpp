#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <random>
#include <queue>
#include <unordered_map>
#include <cmath>
struct node
{
    std::vector<node *> connections;
    cv::Point2f position;
    int failureAttempt = 0;
    bool checkExpansion = false;
};
bool canConnect(int maxDist, node *a, node *b)
{
    double length = cv::norm(a->position - b->position);
    if(length > maxDist)
        return false;
    return true;
}

bool checkIntersection(cv::Point2f start, cv::Point2f end, const cv::Mat &img)
{
    cv::LineIterator it(img, start, end, 8);
    for (int i = 0; i < it.count; i++, it++)
    {
        if (img.at<uchar>(it.pos()) == 0)
        {
            return true; // Intersection found
        }
    }
    return false; // No intersection
}
std::vector<node *> generateNodes(cv::Mat img, int n, int dist, cv::Mat &grid, unsigned int seed)
{
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> xDist(0, img.cols - 1);
    std::uniform_int_distribution<int> yDist(0, img.rows - 1);
    std::vector<node *> nodes;
    for (int i = 0; i < n; i++)
    {

        int xpos = xDist(rng);
        int ypos = yDist(rng);
        cv::Point2f pos(xpos, ypos);
        if (img.at<uchar>(pos) == 0)
        {
            i--;
            continue;
        }
        node *newNode = new node();
        newNode->position = cv::Point2f(xpos, ypos);
        nodes.push_back(newNode);
        grid.at<uchar>(pos) = 1;
    }
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (i != j)
            {
                if (canConnect(dist, nodes[i], nodes[j]))
                {
                    if (!checkIntersection(nodes[i]->position, nodes[j]->position, img))
                    {
                        nodes[i]->connections.push_back(nodes[j]);
                        nodes[j]->connections.push_back(nodes[i]);
                    }
                    else
                    {
                        nodes[i]->failureAttempt++;
                        nodes[j]->failureAttempt++;
                    }
                }
            }
        }
    }
    return nodes;
}
std::vector<node *> expansion(std::vector<node *> nodes, int n, cv::Mat img, cv::Mat grid, int range = 5)
{
    std::vector<cv::Point2f> directions = {
        cv::Point2f(range, 0), cv::Point2f(0, range), cv::Point2f(-range, 0), cv::Point2f(0, -range),
        cv::Point2f(range, range), cv::Point2f(-range, -range), cv::Point2f(range, -range), cv::Point2f(-range, range)};
        std::cout << nodes.size() << std::endl;
        for (int i = 0; i < n; i++)
        {
            if (nodes[i]->connections.size() < 8)
            {
                bool nodeAdded = true;
                
                for (int j = 0; j < directions.size(); j++)
                {
                    cv::Point2f tryPos = nodes[i]->position + directions[j];
                    
                    // Check bounds
                    if (tryPos.x >= 0 && tryPos.x < img.cols && tryPos.y >= 0 && tryPos.y < img.rows)
                    {
                        // Check if the position is free in the grid and not an obstacle
                        if (img.at<uchar>(tryPos) != 0 && grid.at<uchar>(tryPos) != 1 && grid.at<uchar>(tryPos) != 2 )
                        {
                            for (auto* node : nodes[i]->connections)
                            {
                                if(cv::norm(node->position - tryPos) < (range/2))
                                {
                                    nodeAdded = false;
                                    break;
                                }
                            }
                            if (!checkIntersection(nodes[i]->position, tryPos, img) && nodeAdded)
                            {
                                
                                node *newNode = new node();
                                newNode->position = tryPos;
                                newNode->checkExpansion = true;
                                nodes.push_back(newNode);
                                grid.at<uchar>(tryPos) = 2;
                                newNode->connections.push_back(nodes[i]);
                                nodes[i]->connections.push_back(newNode);
                                nodeAdded = true;
                                n--;
                                // Create connections for the new node
                                for (int k = 0; k < nodes.size(); k++)
                                {
                                    if (newNode != nodes[k] && canConnect(range*2, newNode, nodes[k]) && nodes[k]->connections.size() < 8 && k != i)
                                    {
                                        if (!checkIntersection(newNode->position, nodes[k]->position, img))
                                        {
                                            newNode->connections.push_back(nodes[k]);
                                            nodes[k]->connections.push_back(newNode);
                                        }
                                        else
                                        {
                                            newNode->failureAttempt++;
                                            nodes[k]->failureAttempt++;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                if (!nodeAdded) 
                    continue; // Continue checking other directions if no node was added
            }
        }
    return nodes;
}
struct NodeComparator {
    bool operator()(std::pair<node*, double> a, std::pair<node*, double> b) {
        return a.second > b.second; // Smaller costs have higher priority
    }
};

std::vector<cv::Point2f> pathAStar(std::vector<node*>& nodes, unsigned int seed, cv::Mat& img) {
    if (nodes.size() < 2) {
        std::cerr << "Error: Not enough nodes to select start and end points!" << std::endl;
        return {};
    }

    // Randomly select start and end nodes
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> dist(0, nodes.size() - 1);
    node* start = nodes[dist(rng)];
    node* end;
    do {
        end = nodes[dist(rng)];
    } while (end == start);

    // A* setup
    std::priority_queue<std::pair<node*, double>, std::vector<std::pair<node*, double>>, NodeComparator> openList;
    std::unordered_map<node*, double> gCost;  // Cost from start to a node
    std::unordered_map<node*, node*> cameFrom; // Path reconstruction

    // Initialize costs
    for (auto* n : nodes) {
        gCost[n] = std::numeric_limits<double>::infinity();
    }
    gCost[start] = 0.0;

    openList.push({start, 0.0});

    // A* main loop
    while (!openList.empty()) {
        node* current = openList.top().first;
        openList.pop();

        // If we reached the goal
        if (current == end) {
            std::vector<cv::Point2f> path;
            while (current) {
                path.push_back(current->position);
                current = cameFrom[current];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (node* neighbor : current->connections) {
            double tentativeGCost = gCost[current] + cv::norm(current->position - neighbor->position);
            if (tentativeGCost < gCost[neighbor]) {
                gCost[neighbor] = tentativeGCost;
                double fCost = tentativeGCost + cv::norm(neighbor->position - end->position); // Heuristic
                openList.push({neighbor, fCost});
                cameFrom[neighbor] = current;
            }
        }
    }

    std::cerr << "Error: No path found between selected start and end points!" << std::endl;
    return {};
}
void duplicateFrames(cv::Mat& img, cv::VideoWriter& video, int totalFrames) {
    for (int i = 0; i < totalFrames; ++i) {
        video.write(img); // Write the same frame repeatedly
    }
}

void drawPath(const std::vector<cv::Point2f>& path, cv::Mat& img, cv::VideoWriter& video) {
    if (path.size() < 2) {
        std::cerr << "Error: Path is too short to draw!" << std::endl;
        return;
    }

    cv::Scalar blueColor = cv::Scalar(255, 0, 0);
    cv::circle(img, path.front(), 5, cv::Scalar(0, 255, 255), -1); // Start - Yellow
    cv::circle(img, path.back(), 5, cv::Scalar(0, 255, 255), -1);  // End - Yellow
    for (size_t i = 0; i < path.size() - 1; i++) {
        cv::line(img, path[i], path[i + 1], blueColor, 1);
        video.write(img); // Write frame after each segment
    }
    int minFrames = 30; // For a 3-second video at 10 fps
    duplicateFrames(img, video, minFrames - path.size());
}

void drawConnections(std::vector<node *> nodes, cv::Mat &img,cv::Mat &grid, cv::VideoWriter &video)
{
    cv::Scalar greenColor = cv::Scalar(0, 255, 0);
    cv::Scalar redColor = cv::Scalar(0, 0, 255);
    for (int i = 0; i < nodes.size(); i++)
    {
        if(grid.at<uchar>(nodes[i]->position) == 1)
            cv::circle(img, nodes[i]->position, 2, redColor, -1);
        else if(grid.at<uchar>(nodes[i]->position) == 2)
            cv::circle(img, nodes[i]->position, 2, greenColor, -1);
        for (int j = 0; j < nodes[i]->connections.size(); j++)
        {
            if(!nodes[i]->checkExpansion && !nodes[i]->connections[j]->checkExpansion)
                cv::line(img, nodes[i]->position, nodes[i]->connections[j]->position, redColor, 1);
            else
                cv::line(img, nodes[i]->position, nodes[i]->connections[j]->position, greenColor, 1);
        }
        video.write(img);
    }
}

void printData(std::vector<node*>& nodes)
{
    for (int i = 0; i < nodes.size(); i++)
    {
        double length = 0;
        for (int j = 0; j < nodes[i]->connections.size(); j++)
        {
            length += cv::norm(nodes[i]->position - nodes[i]->connections[j]->position);
        }
        length = length / nodes[i]->connections.size();
        std::cout << "Node " << i << " has " << nodes[i]->connections.size() << " connections and " << nodes[i]->failureAttempt << " failed attempts" << " and average length " << length << std::endl;
    }
}
int main(int, char **)
{
    std::string filename3 = "maze.jpg";
    std::string filename1 = "prg_ex2_map.png";
    std::string filename2 = "test_billede.jpg";
    std::string filename4 = "maze2.png";
    cv::Mat img = cv::imread(filename4, cv::IMREAD_GRAYSCALE);
    cv::Mat imgColor = cv::imread(filename4, cv::IMREAD_COLOR);
    cv::Mat imgCopy = img.clone();
    cv::Mat imgNodes = imgColor.clone();
    cv::Mat imgNodesExpansion = imgColor.clone();
    cv::Mat grid = cv::Mat::zeros(img.size(), img.type());
    // Creating videoes for each stage
    cv::VideoWriter videoNodes("nodes.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, imgNodes.size());
  
    int length = 100;
    int seed = 15;
    std::vector<node *> nodes = generateNodes(img, 600, length, grid, seed);
    drawConnections(nodes,imgNodes,grid,videoNodes);
    auto expandedNode = expansion(nodes, 300, img, grid, length/2);
    nodes.insert(nodes.end(), expandedNode.begin(), expandedNode.end());
    // removeLongConnections(finalNodes,length);
    cv::VideoWriter videoNodesExpansion("nodesExpansion.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, imgNodesExpansion.size());
    cv::VideoWriter videoPath("path.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 2, imgNodesExpansion.size());
    drawConnections(nodes, imgNodesExpansion,grid,videoNodesExpansion);
    //printData(nodes);
    cv::Mat imgPath = imgNodesExpansion.clone();
    cv::imwrite("Expansion_picture.jpg",imgNodesExpansion);
    std::vector<cv::Point2f> pathPoints = pathAStar(nodes, seed, imgPath);
    if (pathPoints.empty()) {
        std::cerr << "Error: Path generation failed!" << std::endl;
    } else {
        drawPath(pathPoints, imgPath,videoPath);
        cv::imshow("Path with A* Algorithm", imgPath);
        cv::waitKey(0);
    }
    videoPath.release();
    videoNodes.release();
    videoNodesExpansion.release();
    cv::imshow("Nodes", imgNodes);
    cv::imshow("NodesExpansion", imgNodesExpansion);
    cv::imshow("Path", imgPath);
    return 0;
}
