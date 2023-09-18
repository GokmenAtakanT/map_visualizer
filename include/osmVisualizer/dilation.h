// Function to apply dilation to the matrix
/*
 * Küçük siyah noktaları filtreleme işlemi yapar.
 */
#include <iostream>
#include <vector>

std::vector<std::vector<int>> applyDilation(std::vector<std::vector<int>>& matrix) {
    int rows = matrix.size();
    int cols = matrix[0].size();

    // Create a copy of the original matrix to store the result
    std::vector<std::vector<int>> resultMatrix(rows, std::vector<int>(cols, 0));

    // Iterate through each element of the matrix
    for (int i = 0; i < rows; i++) {

        for (int j = 0; j < cols; j++) {

            // Apply the dilation operation
            if (matrix[i][j] == 1) {
                for (int ki = -1; ki <= 1; ki++) {
                    for (int kj = -1; kj <= 1; kj++) {
                        int ni = i + ki;
                        int nj = j + kj;
                        // Check boundaries
                        if (ni >= 0 && ni < rows && nj >= 0 && nj < cols) {
                            resultMatrix[ni][nj] = 1;
                        }
                    }
                }
            }
        }
    }

    // Copy the result matrix back to the original matrix
    matrix = resultMatrix;
    return matrix;
}