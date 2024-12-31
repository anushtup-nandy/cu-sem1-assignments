from BaseAI import BaseAI
import time
import math

class IntelligentAgent(BaseAI):
    def __init__(self):
        # Maximum depth for search
        self.max_depth = 4
        # Time limit in seconds
        self.time_limit = 0.2
        self.start_time = 0

    def getMove(self, grid):
        self.start_time = time.time()
        return self.expectiminimax(grid, True, float('-inf'), float('inf'), 0)[1]

    def expectiminimax(self, grid, maximizing_player, alpha, beta, depth):
        if depth >= self.max_depth or time.time() - self.start_time > self.time_limit:
            return self.evaluate(grid), None

        if maximizing_player:
            # Player's turn (MAX)
            max_utility = float('-inf')
            best_move = None
            
            for move, new_grid in grid.getAvailableMoves():
                utility = self.expectiminimax(new_grid, False, alpha, beta, depth + 1)[0]
                
                if utility > max_utility:
                    max_utility = utility
                    best_move = move
                
                alpha = max(alpha, max_utility)
                if beta <= alpha:
                    break
                    
            return max_utility, best_move
        else:
            # Computer's turn (CHANCE)
            available_cells = grid.getAvailableCells()
            if not available_cells:
                return self.evaluate(grid), None

            avg_utility = 0
            prob_2 = 0.9  # 90% chance of spawning 2
            prob_4 = 0.1  # 10% chance of spawning 4
            
            for cell in available_cells:
                # Try placing a 2
                new_grid = grid.clone()
                new_grid.insertTile(cell, 2)
                utility_2 = self.expectiminimax(new_grid, True, alpha, beta, depth + 1)[0]
                
                # Try placing a 4
                new_grid = grid.clone()
                new_grid.insertTile(cell, 4)
                utility_4 = self.expectiminimax(new_grid, True, alpha, beta, depth + 1)[0]
                
                # Weighted average based on probabilities
                avg_utility += (utility_2 * prob_2 + utility_4 * prob_4) / len(available_cells)

            return avg_utility, None

    def evaluate(self, grid):
        # Heuristic weights
        EMPTY_WEIGHT = 1
        MONOTONICITY_WEIGHT = 0.7
        SMOOTHNESS_WEIGHT = 0.9
        MAX_TILE_WEIGHT = 1.0

        empty_cells = len(grid.getAvailableCells())
        monotonicity = self.get_monotonicity(grid)
        smoothness = self.get_smoothness(grid)
        max_tile = self.get_max_tile(grid)

        return (EMPTY_WEIGHT * empty_cells +
                MONOTONICITY_WEIGHT * monotonicity +
                SMOOTHNESS_WEIGHT * smoothness +
                MAX_TILE_WEIGHT * max_tile)

    def get_monotonicity(self, grid):
        # Measure how well the tiles are ordered
        scores = [0, 0, 0, 0]  # left, right, up, down
        
        # Check rows
        for i in range(4):
            current = 0
            next_val = current + 1
            while next_val < 4:
                while next_val < 4 and not grid.map[i][next_val]:
                    next_val += 1
                if next_val >= 4:
                    next_val -= 1
                current_value = math.log2(grid.map[i][current]) if grid.map[i][current] else 0
                next_value = math.log2(grid.map[i][next_val]) if grid.map[i][next_val] else 0
                if current_value > next_value:
                    scores[0] += next_value - current_value
                elif current_value < next_value:
                    scores[1] += current_value - next_value
                current = next_val
                next_val += 1
                
        # Check columns
        for i in range(4):
            current = 0
            next_val = current + 1
            while next_val < 4:
                while next_val < 4 and not grid.map[next_val][i]:
                    next_val += 1
                if next_val >= 4:
                    next_val -= 1
                current_value = math.log2(grid.map[current][i]) if grid.map[current][i] else 0
                next_value = math.log2(grid.map[next_val][i]) if grid.map[next_val][i] else 0
                if current_value > next_value:
                    scores[2] += next_value - current_value
                elif current_value < next_value:
                    scores[3] += current_value - next_value
                current = next_val
                next_val += 1
                
        return max(scores[0], scores[1]) + max(scores[2], scores[3])

    def get_smoothness(self, grid):
        smoothness = 0
        for i in range(4):
            for j in range(4):
                if grid.map[i][j]:
                    value = math.log2(grid.map[i][j])
                    # Check right neighbor
                    if j < 3 and grid.map[i][j+1]:
                        smoothness -= abs(value - math.log2(grid.map[i][j+1]))
                    # Check bottom neighbor
                    if i < 3 and grid.map[i+1][j]:
                        smoothness -= abs(value - math.log2(grid.map[i+1][j]))
        return smoothness

    def get_max_tile(self, grid):
        max_tile = 0
        for i in range(4):
            for j in range(4):
                max_tile = max(max_tile, grid.map[i][j])
        return math.log2(max_tile) if max_tile else 0

# from BaseAI import BaseAI
# import sys

# sys.setrecursionlimit(2000)

# class IntelligentAgent(BaseAI):
#     def getMove(self, grid):
#         (child, trash) = self.maximize(grid, -(float("inf")), float("inf"))
#         # print(child)
#         return child[0]
    
#     def minimize(self, grid, alpha, beta):
#         if len(grid.getAvailableMoves()) == 0:
#             return None, self.evaluation(grid)
#         # visited = []
#         (minChild, minUtility) = (None, float("inf"))
#         for child in grid.getAvailableMoves():
#             # if visited.count(child) > 0:
#             #     break
#             childGrid = child[1]
#             (trash, utility) = self.maximize(childGrid, alpha, beta)
#             if utility < minUtility:
#                 (minChild, minUtility) = (child, utility)
#             if minUtility < beta:
#                 print("old", beta)
#                 beta = minUtility
#                 print("new", beta)
#             if minUtility <= alpha:
#                 break
            
#             # visited.append(child)
#         return (minChild, minUtility)

    
#     def maximize(self, grid, alpha, beta):
#         # for i in range(grid.size):
#         #     for j in range(grid.size):
#         #         print(grid.getCellValue((i,j)))
#         if len(grid.getAvailableMoves()) == 0:
#             print("trip")
#             return None, self.evaluation(grid)
#         (maxChild, maxUtility) = (None, -(float("inf")))
#         # visited = []
#         for child in grid.getAvailableMoves():
#             print('ASdosndvo;aed;i;i;aedarhg;ij')
#             print(child)
#             # if visited.count(child) > 0:
#             #     break
#             childGrid = child[1]
#             (trash, utility) = self.minimize(childGrid, alpha, beta)
#             if utility > maxUtility:
#                 (maxChild, maxUtility) = (child, utility)
#             if maxUtility > alpha:
#                 print("old", alpha)
#                 alpha = maxUtility
#                 print("new", alpha)
#             if maxUtility >= beta:
#                 break
            
#             # visited.append(child)
#         print((maxChild, maxUtility))
#         return (maxChild, maxUtility)
    
#     def totalValueHeuristic(self, grid):
#         retVal = 0
#         for i in range(grid.size):
#             for j in range(grid.size):
#                 retVal += grid.getCellValue((i, j))
#         return retVal
    
#     # def heuristic2(self, grid):
#     #     retVal = 0
#     #     for i in range(grid.size):
#     #         for j in range(grid.size):

    
#     def evaluation(self, grid):
#         h1 = self.totalValueHeuristic(grid)

#         return h1 * 1