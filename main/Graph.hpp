//
// Created by max on 18/08/25.
//

#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <unordered_set>
#include <opencv2/opencv.hpp>

using namespace cv;

// use Point as key in hashmap
template<>
struct std::hash<Point> {
    std::size_t operator()(const Point &p) const noexcept {
        return hash<int>()(p.x) ^ (hash<int>()(p.y) << 1);
    }
};

// std::pair<Point, Point> as key in hashmap
// struct PointPairHash {
//     std::size_t operator()(const std::pair<Point, Point> &p) const {
//         return std::hash<Point>()(p.first) ^ (std::hash<Point>()(p.second) << 1);
//     }
// };


enum NodeType {
    NONE,
    ENDPOINT,
    INTERSECTION,
};

struct GraphNode {
    Point pos; // not sure if this is necessary, I am using the pos as key...
    std::unordered_set<GraphNode *> neighbors;
    NodeType type = NONE;
};

struct GraphNodePairHash {
    std::size_t operator()(const std::pair<GraphNode *, GraphNode *> &p) const {
        return std::hash<GraphNode *>()(p.first) ^ (std::hash<GraphNode *>()(p.second) << 1);
    }
};

class GraphPath {
public:
    std::vector<GraphNode *> path;
    std::unordered_set<GraphNode *> nodeset;

    void insert(GraphNode *node);
};

// struct Trail {
//     Point startEndpoint;
//     Point endEndpoint;
//     std::vector<std::pair<Point, Point> > betweenSegmentKeys;
//     std::vector<IntersectionCluster *> intersectionClusters;
//     std::vector<GraphNode *> completePath;
// };
struct Trail {
    GraphNode *start;
    GraphNode *end;
    //TODO: path complete + segments list
};

enum SegmentType {
    BETWEEN,
    END,
    INTER
};

struct Segment {
    //end and in-between segments
    GraphNode *start;
    GraphNode *end;
    std::vector<GraphNode *> path;
    SegmentType type;
    GraphNode *startInter = nullptr;
    GraphNode *endInter = nullptr;
};

struct ISegment {
    // intersection segments
    std::unordered_set<GraphNode *> nodeset;
    SegmentType type = INTER;
    std::unordered_set<Segment *> connectingSegments; // set of Segments the intersection segment connects to
};

using SegmentGroup = std::vector<Segment *>;
using BoolFlags = std::vector<bool>;
using SegmentResult = std::pair<SegmentGroup, BoolFlags>;
using ResultList = std::vector<SegmentResult>;


inline static const std::vector<Point> offsets8 = {
    //
    {-1, -1}, {-1, 0}, {-1, 1},
    {0, -1}, {0, 1},
    {1, -1}, {1, 0}, {1, 1}
};

class Graph {
    // using IntersectionCluster = std::vector<GraphNode>;


    std::unordered_map<Point, GraphNode> nodes;
    std::unordered_set<GraphNode *> endpoints;
    std::unordered_set<GraphNode *> intersections;

    std::unordered_map<GraphNode *, Segment> eSegments;
    std::unordered_map<GraphNode *, Segment *> eSegmentsReverse;
    // this is internal
    std::unordered_map<std::pair<GraphNode *, GraphNode *>, Segment, GraphNodePairHash> _bSegments;
    std::unordered_map<GraphNode *, Segment *> bSegments;

    std::vector<ISegment> iSegments;
    // std::unordered_set<std::reference_wrapper<IntersectionCluster>> intersectionClusters;

    void handleSpecialNode(GraphNode &node);


    static std::vector<GraphNode *> cycleDFS(GraphNode &node, const GraphNode *parent,
                                             std::unordered_set<GraphNode *> &visited,
                                             GraphPath &path, int thresh = 6);

public:
    void buildGraph(const Mat &img, int lbl);


    static std::vector<GraphNode *> cycleDFS_Iterative(GraphNode &startNode, std::unordered_set<GraphNode *> &visited,
                                                int thresh);

    void remove(GraphNode &node);

    void removeNodes(const std::vector<GraphNode *> &list);

    void removeNodes(const std::vector<GraphNode *> &list, Mat &img);

    // only very small cycles < 3, useful after building graph and after removing spurs
    // ensures we do not have intersection points on a straight line between endpoints induced by skeletonization
    void pruneCycles(Mat &img);

    void getSegments();

    ResultList dfs(GraphNode *currentPos, std::vector<Segment *> path, std::vector<bool> reversePath, Segment *
                   endSeg);

    void decomposeTrails(Mat &img, Mat &hsv);

    // void dfs();


    static std::vector<GraphNode *> getEndSegment(GraphNode &node, bool includeIntersection = false);

    std::vector<GraphNode *> getEndpoints() const;

    // std::vector<IntersectionCluster> getIntersectionClusters() const;
};

#endif //GRAPH_HPP
