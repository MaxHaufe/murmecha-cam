//
// Created by max on 18/08/25.
//

#include "Graph.hpp"

#include <csignal>
#include <ranges>
#include <iostream>
#include <queue>
#include <stack>

#include "CurveDir.hpp"
#include "RingBuffer.hpp"
#include "hungarian.hpp"

void Graph::buildGraph(const Mat &img, const int lbl) {
    //clear
    endpoints.clear();
    intersections.clear();
    // intersectionClusters.clear();
    nodes.clear();

    // TODO: this could be optimized if we build all graphs at once instead iterating through the image for every label
    for (int y = 0; y < img.rows; y++) {
        for (int x = 0; x < img.cols; x++) {
            if (img.at<uint8_t>(y, x) == lbl) {
                Point p(x, y);
                const GraphNode n(p, {}); // leave neighbors empty for now
                nodes[p] = n;
            }
        }
    }

    //neighbors
    for (auto &[p, node]: nodes) {
        for (const auto &offset: offsets8) {
            // all 8 surrounding neighbors
            const Point nbr = p + offset;
            if (nodes.contains(nbr)) {
                node.neighbors.insert(&nodes[nbr]);
            }
        }

        handleSpecialNode(node);
    }
}

void GraphPath::insert(GraphNode *node) {
    path.push_back(node);
    nodeset.insert(node);
}

void Graph::handleSpecialNode(GraphNode &node) {
    // endpoint or intersection?
    if (node.neighbors.size() == 1) {
        // endpoint
        endpoints.insert(&node);
        node.type = ENDPOINT;
    }
    if (node.neighbors.size() >= 3) {
        intersections.insert(&node);
        node.type = INTERSECTION;
    }
}

std::vector<GraphNode *> Graph::cycleDFS(GraphNode &node, const GraphNode *parent,
                                         std::unordered_set<GraphNode *> &visited, GraphPath &path, const int thresh) {
    visited.insert(&node);
    path.insert(&node); // TODO: maybe dont use insert but do it here
    // path set?
    std::vector<GraphNode *> rm;


    for (const auto nbr: node.neighbors) {
        if (nbr == parent) {
            continue; //only go forward
        }
        if (path.nodeset.contains(nbr)) {
            // cycle found

            auto it = std::ranges::find(path.path, nbr);
            //we can assume the element is in path, since it is in the nodeset
            auto restPath = std::ranges::subrange(std::next(it), path.path.end());
            const size_t len = std::ranges::size(restPath);

            // std::cout << "Cycle found between " << node.pos << " and " << nbr->pos << " of len " << len << std::endl;

            if (len < thresh) {
                for (const auto cNode: restPath) {
                    // only remove non-intersection points
                    if (cNode->type != INTERSECTION) {
                        rm.push_back(cNode);
                        std::cout << "Would delete node " << cNode->pos << std::endl;
                    }
                }
            }
        } else if (!visited.contains(nbr)) {
            auto ret = cycleDFS(*nbr, &node, visited, path);
            rm.insert(rm.end(), ret.begin(), ret.end());
        }
    }
    path.nodeset.erase(&node);
    path.path.pop_back();
    return rm;
}

struct DFSState {
    GraphNode *node;
    GraphNode *parent;
    std::unordered_set<GraphNode *>::iterator neighbor_it;
    bool processing;

    DFSState(GraphNode *n, GraphNode *p) : node(n), parent(p), neighbor_it(n->neighbors.begin()), processing(false) {
    }
};

std::vector<GraphNode *> Graph::cycleDFS_Iterative(GraphNode &startNode,
                                                   std::unordered_set<GraphNode *> &visited,
                                                   const int thresh) {
    std::vector<GraphNode *> rm;
    std::stack<DFSState> dfsStack;
    std::vector<GraphNode *> path;
    std::unordered_set<GraphNode *> pathSet;

    dfsStack.emplace(&startNode, nullptr);

    while (!dfsStack.empty()) {
        auto &state = dfsStack.top();

        if (!state.processing) {
            // First time visiting this node
            visited.insert(state.node);
            path.push_back(state.node);
            pathSet.insert(state.node);
            state.processing = true;
        }

        // Process neighbors
        bool hasUnprocessedNeighbor = false;
        while (state.neighbor_it != state.node->neighbors.end()) {
            auto nbr = *state.neighbor_it;
            ++state.neighbor_it; // Move to next neighbor

            if (nbr == state.parent) {
                continue;
            }

            if (pathSet.contains(nbr)) {
                // Cycle detected
                auto it = std::find(path.begin(), path.end(), nbr);
                if (it != path.end()) {
                    size_t cycleLen = std::distance(std::next(it), path.end());

                    if (cycleLen < thresh) {
                        for (auto iter = std::next(it); iter != path.end(); ++iter) {
                            if ((*iter)->type != INTERSECTION) {
                                rm.push_back(*iter);
                                std::cout << "Would delete node " << (*iter)->pos << std::endl;
                                //
                            }
                        }
                    }
                }
            } else if (!visited.contains(nbr)) {
                // Continue DFS
                dfsStack.emplace(nbr, state.node);
                hasUnprocessedNeighbor = true;
                break;
            }
        }

        if (!hasUnprocessedNeighbor) {
            // Done with this node
            pathSet.erase(state.node);
            if (!path.empty()) path.pop_back();
            dfsStack.pop();
        }
    }

    return rm;
}

void Graph::remove(GraphNode &node) {
    // std::cout << "Rm node: " << node.pos << std::endl;

    if (node.type == ENDPOINT) {
        endpoints.erase(&node);
    } else if (node.type == INTERSECTION) {
        intersections.erase(&node);
    }

    // remove node from neighbors
    for (const auto nbr: node.neighbors) {
        nbr->neighbors.erase(&node);

        // check if now endpoint/intersection
        if (nbr->type == ENDPOINT) {
            // TODO: point now has no neighbors
            nbr->type = NONE;
            endpoints.erase(nbr);
        } else if (nbr->type == INTERSECTION) {
            nbr->type = NONE;
            intersections.erase(nbr);
        }
        handleSpecialNode(*nbr);
    }
    // IMPORTANT: do this last, because if not, we will have danglin ptrs
    nodes.erase(node.pos); // delete from map
}

void Graph::removeNodes(const std::vector<GraphNode *> &list) {
    for (const auto node: list) {
        remove(*node);
    }
}

void Graph::removeNodes(const std::vector<GraphNode *> &list, Mat &img) {
    for (const auto &node: list) {
        remove(*node);
        img.at<uint8_t>(node->pos) = 0;
    }
}

void Graph::pruneCycles(Mat &img) {
    /*
     * the goal here is to prune cycles of intersection nodes.
     */

    std::unordered_set<GraphNode *> visited;
    GraphPath path;
    std::unordered_set<GraphNode *> rm;

    for (auto node: intersections) {
        // only search intersections as start points
        if (!visited.contains(node)) {
            // auto ret = cycleDFS(*node, nullptr, visited, path, 6);
            auto ret = cycleDFS_Iterative(*node, visited, 6);
            for (const auto &n: ret) {
                rm.insert(n);
            }
            // rm.insert(rm.end(), ret.begin(), ret.end());
        }
    }

    // check for duplicates

    // std::cout << "Removing " << rm.size() << " nodes" << std::endl;
    const std::vector<GraphNode *> rmVec(rm.begin(), rm.end());
    removeNodes(rmVec, img);
}


void Graph::getSegments() {
    //even number of endpoints
    // break apart trail into segments: endsegments, intersectionsegments, inbetweensegments

    // minimization problem.

    // match end segments together
    /*
    * Divide the graph into:
    * endsegments (endpoint -> intersection)
    * intersectionsegments (set of intersectionpoints that are connected), might be owned by multiple
    * betweensegments (line where every node has exactly two neighbors, starts and ends at intersection point), might be owned by multiple
     */

    // we assume an even number of endpoints
    // we can search the endSegments by endpoint.
    // allows us to get eSegment by last element
    // this is only for faster lookup

    eSegments.clear();
    eSegmentsReverse.clear();
    bSegments.clear();
    iSegments.clear();

    for (const auto ep: endpoints) {
        auto path = getEndSegment(*ep, true);
        // last element is the intersection node
        GraphNode *iNode = path.back();
        path.pop_back();
        Segment seg(ep, path.back(), path, END, nullptr, iNode);
        eSegments[ep] = seg;
        eSegmentsReverse[path.back()] = &eSegments[ep];
    }

    // in-between segments
    for (const auto &inode: intersections) {
        for (const auto &nbr: inode->neighbors) {
            if (nbr->type == INTERSECTION || nbr->type == ENDPOINT) {
                continue;
            }

            std::vector<GraphNode *> path;
            GraphNode *current = nbr;
            GraphNode *prev = inode;
            while (current && current->type != INTERSECTION) {
                // visited.push ??? -> no

                if (current->type == ENDPOINT) {
                    //accidentally found an end segment
                    path = {};
                    break;
                }

                path.push_back(current);
                // current has exactly two nbrs
                GraphNode *next = nullptr;
                for (auto inbr: current->neighbors) {
                    // 2 iterations
                    if (inbr != prev) {
                        next = inbr;
                        break;
                    }
                }
                prev = current;
                current = next;
            }
            if (!path.empty()) {
                GraphNode *start = path.front();
                GraphNode *end = path.back();
                Segment seg(start, end, path, BETWEEN, inode, current);
                //TODO double insertions
                _bSegments[std::make_pair(start, end)] = seg;
                bSegments[start] = &_bSegments[std::make_pair(start, end)];
                bSegments[end] = &_bSegments[std::make_pair(start, end)];
            }
        }
    }


    // intersection clusters
    std::vector<std::unordered_set<GraphNode *> > intersectionClusters;
    std::unordered_set<GraphNode *> intersectionVisited;

    for (const auto &intersection: intersections) {
        if (intersectionVisited.contains(intersection)) {
            continue;
        }
        std::unordered_set<GraphNode *> cluster;
        std::queue<GraphNode *> queue;

        queue.push(intersection);
        intersectionVisited.insert(intersection);

        while (!queue.empty()) {
            GraphNode *current = queue.front();
            queue.pop();
            cluster.insert(current);

            for (auto neighbor: current->neighbors) {
                if (neighbor->type == INTERSECTION && !intersectionVisited.contains(neighbor)) {
                    intersectionVisited.insert(neighbor);
                    queue.push(neighbor);
                }
            }
        }
        intersectionClusters.push_back(cluster);
    }


    for (const auto &cluster: intersectionClusters) {
        ISegment iseg;
        iseg.nodeset = cluster;

        // Find neighboring non-intersection points
        for (auto *inode: cluster) {
            for (auto *nbr: inode->neighbors) {
                if (nbr->type != INTERSECTION) {
                    // start node for path
                    if (bSegments.contains(nbr)) {
                        iseg.connectingSegments.insert(bSegments[nbr]); // pointer to between-Segment
                    } else {
                        // if it is not in the a between segments it has to be the last point of an end segment
                        // find endsegment
                        if (eSegmentsReverse.contains(nbr)) {
                            Segment *eseg = eSegmentsReverse[nbr];
                            iseg.connectingSegments.insert(eseg);
                        } else {
                            throw std::runtime_error("iSeg construction");
                        }
                    }
                }
            }
        }
        iSegments.push_back(iseg);
    }
}

double dirScoreSingle(const std::vector<Segment *> &segments, const std::vector<bool> &flags) {
    constexpr size_t nvals = 50;
    RingBuffer<Point> buf(nvals); // TODO : hyperparam...
    auto prev = segments.front();
    for (const auto &pt: prev->path) {
        buf.push(pt->pos);
    }
    // for every segment in a path
    // for 5 segments, we get 4 comparisons
    double totalScore = 0.0;
    int nSegments = 0;
    for (int i = 1; i < segments.size(); i++) {
        Segment *curSeg = segments[i];
        const bool reverse = flags[i];

        std::vector<Point> curPath;


        if (reverse) {
            // https://stackoverflow.com/a/59614852
            for (const auto &node: std::views::reverse(curSeg->path)) {
                curPath.push_back(node->pos);
            }
        } else {
            for (const auto &node: curSeg->path) {
                curPath.push_back(node->pos);
            }
        }

        // take the first nval elements of curPath, take all if curpath is < nval
        // size_t count = std::min(nvals, curPath.size());
        // std::vector clippedCurPath(curPath.begin(), curPath.begin()+count);

        const double prevDir = CurveDirectionCalculator::calculateDirection(buf.get());
        const double segDir = CurveDirectionCalculator::calculateDirection(curPath);
        // TODO: take only the first x points maybe?? -> works well without
        // const double segDir = CurveDirectionCalculator::calculateDirection(clippedCurPath); // TODO: take only the first x points maybe??

        double directionDiff = std::abs(segDir - prevDir);
        double score = std::exp(-directionDiff);

        totalScore += score;
        nSegments++;

        // insert into buf
        for (const auto &p: curPath) {
            buf.push(p);
        }
    }
    return nSegments > 0 ? totalScore / nSegments : 0.0;
}

std::vector<double> dirScore(ResultList &res) {
    std::vector<double> ret;
    // for every path....
    for (auto [segments, flags]: res) {
        // score for A->B direction
        double forwardScore = dirScoreSingle(segments, flags);

        // score for B->A direction (reverse segments and flags)
        SegmentGroup reversedSegments(segments.rbegin(), segments.rend());
        BoolFlags reversedFlags;
        for (int i = flags.size() - 1; i >= 0; i--) {
            reversedFlags.push_back(!flags[i]); // flip the rest
        }

        double backwardScore = dirScoreSingle(reversedSegments, reversedFlags);

        ret.push_back(std::max(forwardScore, backwardScore));
    }
    return ret;
}


void Graph::decomposeTrails(Mat &img, Mat &hsv) {
    getSegments();

    std::unordered_map<std::pair<GraphNode *, GraphNode *>, std::pair<ResultList, std::vector<double> >,
        GraphNodePairHash> results;

    std::vector<WeightedBipartiteEdge> edges;
    // map all Endsegments to an int for hungarian, as the nodes there are ints
    int i = 0;
    std::unordered_map<GraphNode *, int> nodeToInt; // node -> int
    std::unordered_map<int, GraphNode *> intToNode; //int -> node
    for (const auto &[node, seg]: eSegments) {
        nodeToInt[node] = i;
        intToNode[i] = node;
        i++;
    }

    for (auto it1 = eSegments.begin(); it1 != eSegments.end(); ++it1) {
        GraphNode *start = it1->first;
        Segment &startSeg = it1->second;
        for (auto it2 = std::next(it1); it2 != eSegments.end(); ++it2) {
            GraphNode *end = it2->first;
            Segment &endSeg = it2->second;

            GraphNode *currentPos = startSeg.endInter;
            Segment *prevSeg = &startSeg;

            auto ret = dfs(currentPos, {prevSeg}, {false}, &endSeg); // TODO:


            // std::vector<double> score = colorScore(hsv, ret);
            std::vector<double> score = dirScore(ret);
            // TODO: we can keep all the scores. we can also make it mandatory that ALL between segments have to be visited at least once at some point.

            const double maxScore = *std::ranges::max_element(score);
            const int cost = static_cast<int>(maxScore * 100);

            WeightedBipartiteEdge e(nodeToInt[start], nodeToInt[end], -cost);
            WeightedBipartiteEdge e2(nodeToInt[end], nodeToInt[start], -cost);

            edges.push_back(e);
            edges.push_back(e2);

            // TODO: rm
            if (!results.contains(std::make_pair(end, start))) {
                results[std::make_pair(start, end)] = std::make_pair(ret, score);
            }
        }
    }

    auto solution = hungarianMinimumWeightPerfectMatching(eSegments.size(), edges);


    std::unordered_set<int> visited;
    for (int i = 0; i < solution.size(); i++) {
        const int left = i;
        const int right = solution[i];
        if (visited.contains(left) || visited.contains(right)) {
            continue;
        }
        visited.insert(left);
        visited.insert(right);

        GraphNode *lNode = intToNode[left];
        GraphNode *rNode = intToNode[right];
        double maxScore = *std::ranges::max_element(results[std::make_pair(lNode, rNode)].second);

        std::cout << "Optimal Assignment: " << lNode->pos << " & " << rNode->pos << " with score " << maxScore <<
                std::endl;
    }

    int k = 3;
}

ResultList Graph::dfs(GraphNode *currentPos, std::vector<Segment *> path, std::vector<bool> reversePath,
                      Segment *endSeg) {
    // current is an intersection node
    // get currentIntersection object
    const ISegment *currentISeg = nullptr;
    for (auto &iseg: iSegments) {
        if (iseg.nodeset.contains(currentPos)) {
            currentISeg = &iseg;
            break;
        }
    }
    if (!currentISeg) {
        throw std::runtime_error("currentISeg"); // shouldnt happen
    }

    ResultList ret;

    // for all possible ways I could go, do it
    for (const auto &nxtseg: currentISeg->connectingSegments) {
        auto p = path; // copy
        auto rp = reversePath;
        if (std::ranges::find(p, nxtseg) != p.end()) {
            // been here before
            continue;
        }
        if (nxtseg->type == END) {
            // either I have finished or I am wrong
            if (nxtseg == endSeg) {
                //I have finished
                p.push_back(nxtseg);
                //invert last segment path order
                rp.push_back(true);
                return {std::make_pair(p, rp)}; // TODO?
            }
            continue; // I am at the wrong end, skip path
        }


        // get current position
        if (currentISeg->nodeset.contains(nxtseg->startInter)) {
            //normal
            currentPos = nxtseg->endInter;
            rp.push_back(false);
        } else {
            // reverse path
            currentPos = nxtseg->startInter;
            rp.push_back(true);
        }
        p.push_back(nxtseg); // TODO: not sure
        auto result = dfs(currentPos, p, rp, endSeg);

        bool isEmpty = true;
        for (const auto &v: result) {
            if (!v.first.empty()) {
                isEmpty = false;
            }
        }

        if (!isEmpty) {
            ret.insert(ret.end(), result.begin(), result.end());
        }
    }
    return ret;
}


// walks to intersection points from
std::vector<GraphNode *> Graph::getEndSegment(GraphNode &node, const bool includeIntersection) {
    std::vector<GraphNode *> ret;
    GraphNode *current = &node;
    GraphNode *prev = nullptr;


    while (true) {
        if (!current) {
            // TODO: throw
            throw std::runtime_error("current is nullptr");
        }

        if (current->type == INTERSECTION) {
            break;
        }
        // Important: the intersection is not part of the path
        ret.push_back(current);
        GraphNode *next = nullptr;
        for (const auto nbr: current->neighbors) {
            if (nbr != prev) {
                next = nbr;
                break;
            }
        }

        if (!next) {
            // This should never happen
            break;
        }

        if (next->type == ENDPOINT) {
            return {};
        }

        prev = current;
        current = next;
    }
    if (includeIntersection) {
        ret.push_back(current);
    }

    return ret;
}

// return vec instead of set because faster ?!!?!
std::vector<GraphNode *> Graph::getEndpoints() const {
    std::vector ret(endpoints.begin(), endpoints.end());
    return ret;
}
