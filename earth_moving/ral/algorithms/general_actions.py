import numpy as np
from abc import ABC, abstractmethod

class GeneralActions:
    
    def __init__(self,**kwargs) -> None:
        pass
    
    def generate_aggregates_in_clusters(self, min_pos, max_pos, num_clusters, max_per_cluster, max_radius):
        
        """
        Loads a specified number of aggregate objects into clusters within a given position range.

        :param min_pos: List, the [x, y] coordinates of the minimum position for spawning aggregates.
        :param max_pos: List, the [x, y] coordinates of the maximum position for spawning aggregates.
        :param num_clusters: Integer, the number of clusters to create.
        :param max_per_cluster: Integer, the maximum number of aggregates to place in each cluster.
        :param max_radius: Float, the maximum radius around each cluster center to place aggregates.
        """
        
        aggregate_positions = []
        z_fix = 0.5
                
        cluster_centers = []
        for _ in range(num_clusters):
            cluster_center = min_pos + (np.random.rand(2) * (max_pos - min_pos))            
            cluster_center = np.append(cluster_center, z_fix)
            cluster_centers.append(cluster_center)
                          
        for center in cluster_centers:            
            
            # Randomly determine the number of aggregates to place in the cluster
            num_cluster_aggregates = np.random.randint(1, max_per_cluster)
            
            # Spread the aggregates around the cluster center
            for _ in range(num_cluster_aggregates):                
                radius = np.random.rand() * max_radius
                angle = np.random.rand() * 2 * np.pi
                
                # Generate random offset within the radius
                offset_x = radius * np.cos(angle)
                offset_y = radius * np.sin(angle)
                
                # Calculate the position of the aggregate within the cluster    
                pos = np.array(center) + np.array([offset_x, offset_y, z_fix])                        
                aggregate_positions.append(pos)
                
        return aggregate_positions
                                                