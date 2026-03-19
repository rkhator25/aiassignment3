#include <stdio.h>
#include <limits.h>

#define V 8   // Number of cities

// City names
char *cities[V] = {
    "Delhi",
    "Mumbai",
    "Bangalore",
    "Chennai",
    "Kolkata",
    "Hyderabad",
    "Surat",
    "Jaipur"
};

// Function to find city with minimum distance
int minDistance(int dist[], int visited[]) {
    int min = INT_MAX, min_index = -1;

    for (int v = 0; v < V; v++)
        if (!visited[v] && dist[v] <= min)
            min = dist[v], min_index = v;

    return min_index;
}

// Dijkstra Algorithm
void dijkstra(int graph[V][V], int src) {

    int dist[V];
    int visited[V];

    for (int i = 0; i < V; i++) {
        dist[i] = INT_MAX;
        visited[i] = 0;
    }

    dist[src] = 0;

    for (int count = 0; count < V - 1; count++) {

        int u = minDistance(dist, visited);
        visited[u] = 1;

        for (int v = 0; v < V; v++) {
            if (!visited[v] &&
                graph[u][v] &&
                dist[u] != INT_MAX &&
                dist[u] + graph[u][v] < dist[v]) {

                dist[v] = dist[u] + graph[u][v];
            }
        }
    }

    printf("\nShortest Road Distances from %s:\n", cities[src]);
    printf("---------------------------------------\n");

    for (int i = 0; i < V; i++)
        printf("%-12s : %d km\n", cities[i], dist[i]);
}

int main() {

    int graph[V][V] = {

        // Del Mum Ban Che Kol Hyd Ahm Jai
        {  0, 1400, 2150, 2200, 1500, 1600, 950, 280},  // Delhi
        {1400,   0,  980, 1330, 2050,  710,  520,1150}, // Mumbai
        {2150, 980,    0,  350, 1870,  570,1500,2000},  // Bangalore
        {2200,1330, 350,    0, 1660,  630,1800,2100},   // Chennai
        {1500,2050,1870,1660,   0, 1500,2000,1300},     // Kolkata
        {1600, 710, 570, 630,1500,    0,1200,1500},     // Hyderabad
        { 950, 520,1500,1800,2000,1200,   0, 650},      // Surat
        { 280,1150,2000,2100,1300,1500, 650,   0}       // Jaipur
    };

    int source;

    printf("List of Cities:\n");
    for (int i = 0; i < V; i++)
        printf("%d. %s\n", i, cities[i]);

    printf("\nEnter source city number: ");
    scanf("%d", &source);

    if (source < 0 || source >= V) {
        printf("Invalid city selection!\n");
        return 0;
    }

    dijkstra(graph, source);

    return 0;
}
