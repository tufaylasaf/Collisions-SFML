#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdlib> // for abs()

class SpatialGrid
{
public:
    SpatialGrid(float spacing, int maxNumObjects)
        : spacing(spacing), tableSize(2 * maxNumObjects), querySize(0)
    {
        cellStart.resize(tableSize + 1, 0); // +1 for the guard
        cellEntries.resize(maxNumObjects, 0);
        queryIds.resize(maxNumObjects, 0);
    }

    // Converts 2D coordinates to integer grid index based on spacing
    int intCoord(float coord) const
    {
        return static_cast<int>(std::floor(coord / spacing));
    }

    // Fantasy hash function based on integer 2D coordinates
    int hashCoords(int xi, int yi) const
    {
        int h = (xi * 92837111) ^ (yi * 689287499);
        return std::abs(h) % tableSize;
    }

    // Hashes the position of the object at index `nr`
    int hashPos(const std::vector<float> &pos, int nr) const
    {
        return hashCoords(
            intCoord(pos[2 * nr]),
            intCoord(pos[2 * nr + 1]));
    }

    // Creates the spatial hash grid for 2D positions
    void create(const std::vector<float> &pos)
    {
        int numObjects = std::min(static_cast<int>(pos.size() / 2), static_cast<int>(cellEntries.size()));

        // Reset cell start counts
        std::fill(cellStart.begin(), cellStart.end(), 0);

        // Count objects per hash cell
        for (int i = 0; i < numObjects; ++i)
        {
            int h = hashPos(pos, i);
            cellStart[h]++;
        }

        // Compute start indices for each cell
        int start = 0;
        for (int i = 0; i < tableSize; ++i)
        {
            int temp = cellStart[i];
            cellStart[i] = start;
            start += temp;
        }
        cellStart[tableSize] = start; // Guard

        // Fill in the object indices
        for (int i = 0; i < numObjects; ++i)
        {
            int h = hashPos(pos, i);
            cellStart[h]--;
            cellEntries[cellStart[h]] = i;
        }
    }

    // Queries the spatial hash grid for objects within maxDist of the given object in 2D
    void query(const std::vector<float> &pos, int nr, float maxDist)
    {
        int x0 = intCoord(pos[2 * nr] - maxDist);
        int y0 = intCoord(pos[2 * nr + 1] - maxDist);

        int x1 = intCoord(pos[2 * nr] + maxDist);
        int y1 = intCoord(pos[2 * nr + 1] + maxDist);

        querySize = 0;

        // Iterate over neighboring grid cells within the query range in 2D
        for (int xi = x0; xi <= x1; ++xi)
        {
            for (int yi = y0; yi <= y1; ++yi)
            {
                int h = hashCoords(xi, yi);
                int start = cellStart[h];
                int end = cellStart[h + 1];

                // Add the object indices in the current cell to query results
                for (int i = start; i < end; ++i)
                {
                    queryIds[querySize++] = cellEntries[i];
                }
            }
        }
    }

    // Returns the result of the last query
    const std::vector<int> &getQueryResults() const
    {
        return queryIds;
    }

    // Returns the number of objects in the result of the last query
    int getQuerySize() const
    {
        return querySize;
    }

    // Renders the grid to the given window
    void renderGrid(sf::RenderWindow &window)
    {
        // Set color and thickness for grid lines
        sf::Color gridColor(100, 100, 100, 150); // Semi-transparent gray
        const float thickness = 1.0f;

        // Vertical lines
        for (float x = 0; x < window.getSize().x; x += spacing)
        {
            sf::Vertex line[] =
                {
                    sf::Vertex(sf::Vector2f(x, 0), gridColor),
                    sf::Vertex(sf::Vector2f(x, window.getSize().y), gridColor)};
            window.draw(line, 2, sf::Lines);
        }

        // Horizontal lines
        for (float y = 0; y < window.getSize().y; y += spacing)
        {
            sf::Vertex line[] =
                {
                    sf::Vertex(sf::Vector2f(0, y), gridColor),
                    sf::Vertex(sf::Vector2f(window.getSize().x, y), gridColor)};
            window.draw(line, 2, sf::Lines);
        }
    }

private:
    float spacing;                // Spacing of grid cells
    int tableSize;                // Size of the hash table
    std::vector<int> cellStart;   // Start index of each hash cell
    std::vector<int> cellEntries; // Object indices for each cell
    std::vector<int> queryIds;    // List of results from last query
    int querySize;                // Number of results from last query
};
