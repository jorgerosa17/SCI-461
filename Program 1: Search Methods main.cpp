#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <chrono>

// Structure representing a city
struct City {
    std::string name;
    double longitude;
    double latitude;

    // Default constructor
    City() : longitude(0.0), latitude(0.0) {}

    // Constructor with arguments
    City(const std::string& cityName, double lon, double lat) : name(cityName), longitude(lon), latitude(lat) {}
};

class CityGraph {
public:
    void addEdge(const std::string& city1, const std::string& city2) {
        adjacencyList[city1].push_back(city2);
        adjacencyList[city2].push_back(city1);
    }

    bool hasCity(const std::string& cityName) const {
        return cityCoordinates.find(cityName) != cityCoordinates.end();
    }

    void addCity(const std::string& cityName, double longitude, double latitude) {
        cityCoordinates[cityName] = City(cityName, longitude, latitude);
    }

    void depthFirstSearch(const std::string& startCity) {
        auto startTime = std::chrono::high_resolution_clock::now();
        std::unordered_set<std::string> visited;
        dfsHelper(startCity, visited);
        auto endTime = std::chrono::high_resolution_clock::now();
        printElapsedTime(startTime, endTime);
    }

    void breadthFirstSearch(const std::string& startCity) {
        auto startTime = std::chrono::high_resolution_clock::now();
        std::unordered_set<std::string> visited;
        std::queue<std::string> bfsQueue;

        visited.insert(startCity);
        bfsQueue.push(startCity);

        while (!bfsQueue.empty()) {
            std::string currentCity = bfsQueue.front();
            bfsQueue.pop();

            std::cout << "Visited City: " << currentCity << std::endl;

            for (const std::string& neighbor : adjacencyList[currentCity]) {
                if (visited.find(neighbor) == visited.end()) {
                    visited.insert(neighbor);
                    bfsQueue.push(neighbor);
                }
            }
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        printElapsedTime(startTime, endTime);
    }

    void iterativeDeepeningDepthFirstSearch(const std::string& startCity, int maxDepth) {
        auto startTime = std::chrono::high_resolution_clock::now();
        for (int depth = 0; depth <= maxDepth; ++depth) {
            std::unordered_set<std::string> visited;
            if (idDfsHelper(startCity, visited, depth)) {
                auto endTime = std::chrono::high_resolution_clock::now();
                printElapsedTime(startTime, endTime);
                return; // Stop searching if the goal is found
            }
        }
    }

    void bestFirstSearch(const std::string& startCity, const std::string& goalCity) {
        auto startTime = std::chrono::high_resolution_clock::now();
        std::priority_queue<std::pair<double, std::string>, std::vector<std::pair<double, std::string>>, std::greater<>> pq;
        std::unordered_set<std::string> visited;

        pq.push({heuristic(startCity, goalCity), startCity});

        while (!pq.empty()) {
            auto [cost, currentCity] = pq.top();
            pq.pop();

            if (visited.find(currentCity) != visited.end()) {
                continue;
            }

            visited.insert(currentCity);
            std::cout << "Visited City: " << currentCity << std::endl;

            if (currentCity == goalCity) {
                auto endTime = std::chrono::high_resolution_clock::now();
                printElapsedTime(startTime, endTime);
                return; // Goal reached
            }

            for (const std::string& neighbor : adjacencyList[currentCity]) {
                if (visited.find(neighbor) == visited.end()) {
                    pq.push({heuristic(neighbor, goalCity), neighbor});
                }
            }
        }
    }

    void aStarSearch(const std::string& startCity, const std::string& goalCity) {
        auto startTime = std::chrono::high_resolution_clock::now();
        std::priority_queue<std::pair<double, std::string>, std::vector<std::pair<double, std::string>>, std::greater<>> pq;
        std::unordered_map<std::string, double> costSoFar;
        std::unordered_set<std::string> visited;

        pq.push({0.0, startCity});
        costSoFar[startCity] = 0.0;

        while (!pq.empty()) {
            auto [cost, currentCity] = pq.top();
            pq.pop();

            if (visited.find(currentCity) != visited.end()) {
                continue;
            }

            visited.insert(currentCity);
            std::cout << "Visited City: " << currentCity << std::endl;

            if (currentCity == goalCity) {
                auto endTime = std::chrono::high_resolution_clock::now();
                printElapsedTime(startTime, endTime);
                return; // Goal reached
            }

            // Check if the city exists in the map before using 'at' method
            if (cityCoordinates.find(currentCity) == cityCoordinates.end()) {
                std::cerr << "Error: City coordinates not found for " << currentCity << std::endl;
                return; // Exit the method or handle the error appropriately
            }

            for (const std::string& neighbor : adjacencyList[currentCity]) {
                if (visited.find(neighbor) == visited.end()) {
                    // Check if the neighbor city exists in the map before using 'at' method
                    if (cityCoordinates.find(neighbor) == cityCoordinates.end()) {
                        std::cerr << "Error: City coordinates not found for " << neighbor << std::endl;
                        return; // Exit the method or handle the error appropriately
                    }

                    double newCost = costSoFar[currentCity] + distance(currentCity, neighbor) + heuristic(neighbor, goalCity);

                    // Check if the neighbor city has not been visited or has a lower cost
                    if (costSoFar.find(neighbor) == costSoFar.end() || newCost < costSoFar[neighbor]) {
                        costSoFar[neighbor] = newCost;
                        pq.push({newCost, neighbor});
                    }
                }
            }
        }
    }

    // Function to calculate the memory used (approximate)
    std::size_t getMemoryUsage() const {
        std::size_t memoryUsage = 0;

        for (const auto& cityPair : cityCoordinates) {
            memoryUsage += sizeof(cityPair.second); // Size of City object
            memoryUsage += cityPair.first.capacity(); // Size of string (city name)
        }

        for (const auto& adjacencyPair : adjacencyList) {
            memoryUsage += adjacencyPair.first.capacity(); // Size of string (city name)
            memoryUsage += adjacencyPair.second.size() * sizeof(std::string); // Size of vector of strings
        }

        return memoryUsage;
    }

private:
    std::unordered_map<std::string, std::vector<std::string>> adjacencyList;
    std::unordered_map<std::string, City> cityCoordinates;

    void dfsHelper(const std::string& currentCity, std::unordered_set<std::string>& visited) {
        visited.insert(currentCity);
        std::cout << "Visited City: " << currentCity << std::endl;

        for (const std::string& neighbor : adjacencyList[currentCity]) {
            if (visited.find(neighbor) == visited.end()) {
                dfsHelper(neighbor, visited);
            }
        }
    }

    bool idDfsHelper(const std::string& currentCity, std::unordered_set<std::string>& visited, int depth) {
        visited.insert(currentCity);
        std::cout << "Visited City: " << currentCity << std::endl;

        if (depth == 0) {
            return true; // Goal reached at the specified depth
        }

        for (const std::string& neighbor : adjacencyList[currentCity]) {
            if (visited.find(neighbor) == visited.end()) {
                if (idDfsHelper(neighbor, visited, depth - 1)) {
                    return true; // Goal found in deeper levels
                }
            }
        }

        return false;
    }

    double heuristic(const std::string& city1, const std::string& city2) const {
        // Check if the cities exist in the map before using 'at' method
        if (cityCoordinates.find(city1) != cityCoordinates.end() && cityCoordinates.find(city2) != cityCoordinates.end()) {
            // A simple Euclidean distance as a heuristic
            return distance(city1, city2);
        } else {
            // Return a default value if one of the cities is not found
            return 0.0;
        }
    }

    double distance(const std::string& city1, const std::string& city2) const {
        const City& c1 = cityCoordinates.at(city1);
        const City& c2 = cityCoordinates.at(city2);
        return std::sqrt(std::pow(c1.longitude - c2.longitude, 2) + std::pow(c1.latitude - c2.latitude, 2));
    }

    void printElapsedTime(const std::chrono::high_resolution_clock::time_point& start,
                          const std::chrono::high_resolution_clock::time_point& end) const {
        auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        std::cout << "Time elapsed: " << elapsedTime << " microseconds" << std::endl;
    }
};

int main() {
    // Read adjacency information from "adjacencies.txt"
    std::ifstream adjacenciesFile("adjacencies.txt");
    if (!adjacenciesFile.is_open()) {
        std::cerr << "Error opening adjacencies file!" << std::endl;
        return 1;
    }

    CityGraph cityGraph;

    std::string adjacencyLine;
    while (std::getline(adjacenciesFile, adjacencyLine)) {
        std::istringstream lineStream(adjacencyLine);
        std::string city1, city2;
        lineStream >> city1 >> city2;
        cityGraph.addEdge(city1, city2);
    }

    adjacenciesFile.close();

    // Read coordinates from "coordinates.csv"
    std::ifstream coordinatesFile("coordinates.csv");
    if (!coordinatesFile.is_open()) {
        std::cerr << "Error opening coordinates file!" << std::endl;
        return 1;
    }

    std::string coordinatesLine;

    while (std::getline(coordinatesFile, coordinatesLine)) {
        std::istringstream lineStream(coordinatesLine);
        std::string cityName;
        double longitude, latitude;

        // Assuming CSV-like format with columns: cityName,longitude,latitude
        lineStream >> cityName >> longitude >> latitude;
        cityGraph.addEdge(cityName, ""); // Add an empty neighbor to avoid disconnected nodes
        cityGraph.addCity(cityName, longitude, latitude);
    }

    coordinatesFile.close();

    std::string startCity, goalCity;

    do {
        // Get starting city from the user
        std::cout << "Enter the starting city (or type 'exit' to quit): ";
        std::cin >> startCity;

        if (startCity == "exit") {
            break; // Exit the loop and end the program
        }

//        // Check if the starting city exists in the graph
//        if (!cityGraph.hasCity(startCity)) {
//            std::cout << "City not found. Please enter a valid city name." << std::endl;
//            continue; // Skip the rest of the loop and ask for a new city
//        }

        // Get goal city from the user
        std::cout << "Enter the goal city: ";
        std::cin >> goalCity;

//        if (!cityGraph.hasCity(goalCity)) {
//            std::cout << "Goal city not found. Please enter a valid goal city name." << std::endl;
//            continue; // Skip the rest of the loop and ask for a new goal city
//        }

        // Ask the user for the search method
        std::cout << "Choose the search method:\n";
        std::cout << "1. Depth-First Search (DFS)\n";
        std::cout << "2. Breadth-First Search (BFS)\n";
        std::cout << "3. Iterative Deepening Depth-First Search (ID-DFS)\n";
        std::cout << "4. Best-First Search\n";
        std::cout << "5. A* Search\n";
        std::cout << "Enter the number of your choice: ";

        int searchChoice;
        std::cin >> searchChoice;

        switch (searchChoice) {
            case 1:
                // Perform DFS starting from the user-specified city
                std::cout << "\nDepth-First Search Results:" << std::endl;
                cityGraph.depthFirstSearch(startCity);
                break;
            case 2:
                // Perform BFS starting from the user-specified city
                std::cout << "\nBreadth-First Search Results:" << std::endl;
                cityGraph.breadthFirstSearch(startCity);
                break;
            case 3:
                // Perform ID-DFS starting from the user-specified city with maximum depth 5
                std::cout << "\nIterative Deepening Depth-First Search (ID-DFS) Results:" << std::endl;
                cityGraph.iterativeDeepeningDepthFirstSearch(startCity, 5);
                break;
            case 4:
                // Perform Best-First Search
                std::cout << "\nBest-First Search Results:" << std::endl;
                cityGraph.bestFirstSearch(startCity, goalCity);
                break;
            case 5:
                // Perform A* Search
                std::cout << "\nA* Search Results:" << std::endl;
                cityGraph.aStarSearch(startCity, goalCity);
                break;
            default:
                std::cout << "Invalid choice. Exiting the program." << std::endl;
                return 1; // Exit with an error code
        }

        // Ask the user if they want to try a new city or exit
        std::cout << "\nChoose your next action:\n";
        std::cout << "1. Try a new city and goal\n";
        std::cout << "2. Exit\n";
        std::cout << "Enter the number of your choice: ";

        int choice;
        std::cin >> choice;

        switch (choice) {
            case 1:
                break; // Continue to the next iteration and ask for a new city and goal
            case 2:
                return 0; // Exit the program
            default:
                std::cout << "Invalid choice. Exiting the program." << std::endl;
                return 1; // Exit with an error code
        }

    } while (true);

    // Display approximate memory usage
    std::cout << "Approximate total memory used: " << cityGraph.getMemoryUsage() << " bytes" << std::endl;

    return 0;
}
