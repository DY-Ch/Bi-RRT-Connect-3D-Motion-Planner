import numpy as np


class MyPlanner:
    __slots__ = ["boundary", "blocks", "step_size", "max_iter"]

    def __init__(self, boundary, blocks):
        self.boundary = boundary
        self.blocks = blocks

        # Bi-direc. RRT algo. parameters (fine-tune for Part 3 time constraints)
        self.step_size = 0.4  # Max step size for each Steer operation
        self.max_iter = 100000  # Max number of iterations

    def check_collision(self, p1, p2):
        """
        Part 1: Vectorized Slab Method line-segment collision detection
        Check whether the line segment (from p1 to p2) collides with any AABB obstacle or boundary.
        """
        # 1. Environment boundary check (based on the original boundary 2D array dimensions)
        # 2. Vectorized processing for all AABBs

        O = np.array(p1, dtype=np.float64)
        D = np.array(p2, dtype=np.float64) - O

        # 1. Environment boundary check
        env_min = self.boundary[0, 0:3]
        env_max = self.boundary[0, 3:6]
        if (
            np.any(p1 < env_min)
            or np.any(p1 > env_max)
            or np.any(p2 < env_min)
            or np.any(p2 > env_max)
        ):
            return True

        if len(self.blocks) == 0:
            return False

        # 2. Vectorized processing for all AABBs
        D_safe = np.where(D == 0, 1e-9, D)
        blocks_min = self.blocks[:, 0:3]
        blocks_max = self.blocks[:, 3:6]

        t1 = (blocks_min - O) / D_safe
        t2 = (blocks_max - O) / D_safe

        t_min = np.minimum(t1, t2)
        t_max = np.maximum(t1, t2)

        t_enter = np.max(t_min, axis=1)
        t_exit = np.min(t_max, axis=1)

        collisions = (t_enter <= t_exit) & (t_exit >= 0) & (t_enter <= 1)
        return np.any(collisions)

    def plan(self, start, goal):
        """
        Part 2 & 3: Path planning based on Bi-direc. RRT
        Deal with maze w/ narrow passages and dynamic goals under strict time constraints
        """
        # Preallocate memory for the two trees separately
        nodes_A = np.zeros((self.max_iter, 3))
        nodes_B = np.zeros((self.max_iter, 3))
        parents_A = np.zeros(self.max_iter, dtype=int)
        parents_B = np.zeros(self.max_iter, dtype=int)

        nodes_A[0] = start
        nodes_B[0] = goal
        parents_A[0] = -1
        parents_B[0] = -1

        num_A = 1
        num_B = 1

        env_min = self.boundary[0, 0:3]
        env_max = self.boundary[0, 3:6]

        swapped = False

        # 1. Randomly sample in the space
        # 2. Extend tree A
        # 3. Connect (Tree B greedily extends toward q_new_A)
        # 4. Reconstruct the final path if the two trees connected
        # 5. Swap the two trees (maintain balanced growth)

        for _ in range(self.max_iter):
            if num_A >= self.max_iter - 100 or num_B >= self.max_iter - 100:
                break  # Prevent array out-of-bounds

            # 1. Randomly sample
            q_rand = np.random.uniform(env_min, env_max)

            # 2. Extend tree A
            dists_A = np.sum((nodes_A[:num_A] - q_rand) ** 2, axis=1)
            nearest_A_idx = np.argmin(dists_A)
            q_near_A = nodes_A[nearest_A_idx]

            dir_A = q_rand - q_near_A
            dist_A = np.linalg.norm(dir_A)
            if dist_A > self.step_size:
                q_new_A = q_near_A + (dir_A / dist_A) * self.step_size
            else:
                q_new_A = q_rand

            if not self.check_collision(q_near_A, q_new_A):
                nodes_A[num_A] = q_new_A
                parents_A[num_A] = nearest_A_idx
                num_A += 1

                # 3. Attempt to directly connect tree B to the newly generated q_new_A
                dists_B = np.sum((nodes_B[:num_B] - q_new_A) ** 2, axis=1)
                nearest_B_idx = np.argmin(dists_B)
                q_near_B = nodes_B[nearest_B_idx]

                curr_B = q_near_B
                curr_B_idx = nearest_B_idx
                reached = False

                # Tree B greedily extends toward q_new_A (CONNECT step)
                while True:
                    dir_B = q_new_A - curr_B
                    dist_B = np.linalg.norm(dir_B)

                    if dist_B <= self.step_size:
                        if not self.check_collision(curr_B, q_new_A):
                            nodes_B[num_B] = q_new_A
                            parents_B[num_B] = curr_B_idx
                            num_B += 1
                            reached = True
                        break
                    else:
                        q_step_B = curr_B + (dir_B / dist_B) * self.step_size
                        if not self.check_collision(curr_B, q_step_B):
                            nodes_B[num_B] = q_step_B
                            parents_B[num_B] = curr_B_idx
                            curr_B_idx = num_B
                            num_B += 1
                            curr_B = q_step_B
                        else:
                            break  # Hit an obstacle, stop extending

                # 4. Reconstruct the final path if the two trees connected
                if reached:
                    path_A = []
                    idx = num_A - 1
                    while idx != -1:
                        path_A.append(nodes_A[idx])
                        idx = parents_A[idx]
                    path_A = path_A[::-1]  # Reverse so the start point comes first

                    path_B = []
                    idx = num_B - 1
                    while idx != -1:
                        path_B.append(nodes_B[idx])
                        idx = parents_B[idx]

                    if swapped:
                        # If the trees were swapped: Tree A originates from goal, Tree B from start
                        full_path = path_B[::-1] + path_A[::-1][1:]
                    else:
                        # Normal case: Tree A originates from start, Tree B from goal
                        full_path = path_A + path_B[1:]
                    return np.array(full_path)

            # 5. Swap the two trees
            nodes_A, nodes_B = nodes_B, nodes_A
            parents_A, parents_B = parents_B, parents_A
            num_A, num_B = num_B, num_A
            swapped = not swapped

        print("Warning: RRT-Connect failed.")
        return np.array([start])


class GoalBiasedPlanner:
    __slots__ = ["boundary", "blocks", "step_size", "goal_bias", "max_iter"]

    def __init__(self, boundary, blocks):
        self.boundary = boundary
        self.blocks = blocks

        # RRT algo. parameters (fine-tune for Part 3 time constraints)
        self.step_size = 0.5  # Max step size for each Steer operation
        self.goal_bias = 0.02  # chance to directly sample the goal (faster convergence)
        self.max_iter = 80000  # Max number of iterations

    def check_collision(self, p1, p2):
        """
        Part 1: Vectorized Slab Method line-segment collision detection
        Check whether the line segment (from p1 to p2) collides with any AABB obstacle or boundary.
        """
        # 1. Environment boundary check (based on the original boundary 2D array dimensions)
        # 2. Vectorized processing for all AABBs

        O = np.array(p1, dtype=np.float64)
        D = np.array(p2, dtype=np.float64) - O

        # 1. Environment boundary check
        env_min = self.boundary[0, 0:3]
        env_max = self.boundary[0, 3:6]
        if (
            np.any(p1 < env_min)
            or np.any(p1 > env_max)
            or np.any(p2 < env_min)
            or np.any(p2 > env_max)
        ):
            return True

        if len(self.blocks) == 0:
            return False

        # 2. Vectorized processing for all AABBs
        D_safe = np.where(D == 0, 1e-9, D)
        blocks_min = self.blocks[:, 0:3]
        blocks_max = self.blocks[:, 3:6]

        t1 = (blocks_min - O) / D_safe
        t2 = (blocks_max - O) / D_safe

        t_min = np.minimum(t1, t2)
        t_max = np.maximum(t1, t2)

        t_enter = np.max(t_min, axis=1)
        t_exit = np.min(t_max, axis=1)

        collisions = (t_enter <= t_exit) & (t_exit >= 0) & (t_enter <= 1)
        return np.any(collisions)

    def plan(self, start, goal):
        """
        Part 2 & 3: Path planning based on RRT
        """
        # 1. Sampling (Goal-biased Sampling)
        # 2. Find nearest node (Nearest Neighbor)
        # 3. Extend (Steer)
        # 4. Collision checking
        # 5. Check if the goal region is reached

        # Preallocate NumPy arrays for better performance (avoid repeated list append operations)
        nodes = np.zeros((self.max_iter, 3))
        nodes[0] = start
        parents = [-1] * self.max_iter
        num_nodes = 1

        env_min = self.boundary[0, 0:3]
        env_max = self.boundary[0, 3:6]

        for _ in range(self.max_iter):
            # 1. Sampling (Goal-biased Sampling)
            if np.random.rand() < self.goal_bias:
                q_rand = goal
            else:
                q_rand = np.random.uniform(env_min, env_max)

            # 2. Find nearest node
            # Use NumPy broadcasting to compute squared distances to all existing nodes at once
            dists = np.sum((nodes[:num_nodes] - q_rand) ** 2, axis=1)
            nearest_idx = np.argmin(dists)
            q_near = nodes[nearest_idx]

            # 3. Extend (Steer)
            dir_vec = q_rand - q_near
            dist = np.linalg.norm(dir_vec)
            if dist > self.step_size:
                q_new = q_near + (dir_vec / dist) * self.step_size
            else:
                q_new = q_rand

            # 4. Collision checking
            if not self.check_collision(q_near, q_new):
                nodes[num_nodes] = q_new
                parents[num_nodes] = nearest_idx
                num_nodes += 1

                # 5. Check if the goal region is reached
                if np.linalg.norm(q_new - goal) <= self.step_size:
                    # The final segment also needs collision checking
                    if not self.check_collision(q_new, goal):
                        nodes[num_nodes] = goal
                        parents[num_nodes] = num_nodes - 1
                        num_nodes += 1

                        # Backtrack and reverse the path (from start to goal)
                        path = []
                        curr_idx = num_nodes - 1
                        while curr_idx != -1:
                            path.append(nodes[curr_idx])
                            curr_idx = parents[curr_idx]
                        return np.array(path[::-1])

        print("Warning: RRT failed to find a path within max_iter.")
        return np.array([start])
