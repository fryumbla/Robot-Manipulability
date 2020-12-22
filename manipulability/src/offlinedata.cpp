#include <iostream>

#include <geometry_msgs/Pose.h>

using namespace std;


// stores adjacency list items
struct adjNode {
    geometry_msgs::Pose position;
    float manipulability;
    adjNode* next;
};

// structure to store edges
struct graphEdge {
    geometry_msgs::Pose start_ver, end_ver;
    float weight;
};



class DiaGraph{
    // insert new nodes into adjacency list from given graph
    adjNode* getAdjListNode(geometry_msgs::Pose value, float weight, adjNode* head)   {
        adjNode* newNode = new adjNode;
        newNode->position = value;
        newNode->manipulability = weight;
         
        newNode->next = head;   // point new node to current head
        return newNode;
    }

    int N;  // number of nodes in the graph
    
    public:
        adjNode **head; //adjacency list as array of pointers

        // Constructor
        DiaGraph(graphEdge edges[], int n, int N)  {
            // allocate new node
            head = new adjNode*[N]();
            this->N = N;
            // initialize head pointer for all vertices
            for (int i = 0; i < N; ++i)
                head[i] = nullptr;

            // construct directed graph by adding edges to it
            for (unsigned i = 0; i < n; i++)  {
                geometry_msgs::Pose start_ver = edges[i].start_ver;
                geometry_msgs::Pose end_ver = edges[i].end_ver;
                float weight = edges[i].weight;

                // insert in the beginning
                adjNode* newNode = getAdjListNode(end_ver, weight, head[i]);
                
                // point head pointer to new node
                head[i] = newNode;
                }
        }
        // Destructor
        ~DiaGraph() {
        for (int i = 0; i < N; i++)
            delete[] head[i];
            delete[] head;
        }
};
// print all adjacent vertices of given vertex
void display_AdjList(adjNode* ptr, int i)
{
    while (ptr != nullptr) {
        cout << "[" << "(" << ptr->next->position.position.x << "," << ")" << ", " << "(" << ptr->position.position.x << "," << ptr->position.position.y << ")"
            << ", " << ptr->manipulability << "] ";
        ptr = ptr->next;
    }
    cout << endl;
}
// graph implementation
int main()
{
    geometry_msgs::Pose temp;
    temp.position.x=1;
    temp.position.y=1;


    geometry_msgs::Pose temp1;
    temp1.position.x=2;
    temp1.position.y=2;

    geometry_msgs::Pose temp2;
    temp2.position.x=3;
    temp2.position.y=3;


    // graph edges array.
    graphEdge edges[] = {
        // (x, y, w) -> edge from x to y with weight w
        {temp, temp1,0.2},{temp1,temp2,0.4}
    };
    int N = 2;      // Number of vertices in the graph
    // calculate number of edges
    int n = sizeof(edges)/sizeof(edges[0]);
    // construct graph
    DiaGraph diagraph(edges, n, N);
    // print adjacency list representation of graph
    cout<<"Graph adjacency list "<<endl<<"(start_vertex, end_vertex, weight):"<<endl;
    for (int i = 0; i < N; i++)
    {
        // display adjacent vertices of vertex i
        display_AdjList(diagraph.head[i], i);
    }
    return 0;
}
