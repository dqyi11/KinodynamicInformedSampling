import plot_utils
import distance_functions

if __name__ == '__main__':

    plot_utils.plot_surf((0,10), (0,10), (20,20), \
                            distance_functions.L2_distance)
