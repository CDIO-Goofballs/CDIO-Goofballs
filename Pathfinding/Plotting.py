import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon as MplPolygon, Circle
import numpy as np

def plot_route(start, vip, others, end, obstacles, safe_points, full_path, best_order, has_vip, width,
               height, ball_diameter=4, original_obstacles=None, debug=False, start_angle=0):
    if debug:
        fig, ax = plt.subplots()
    else:
        plt.ion() # Interactive mode
        plt.clf()
        fig, ax = plt.gcf(), plt.gca()

    fig.set_size_inches(8, 6, True)
    radius = ball_diameter / 2

    if original_obstacles is None:
        original_obstacles = obstacles

    # Plot inflated obstacles with original ones inside
    for inflated_obs, original_obs in zip(obstacles, original_obstacles):
    # Inflated obstacle (outer)
        inflated_patch = MplPolygon(list(inflated_obs.exterior.coords), closed=True, facecolor='lightgray', edgecolor='gray', alpha=0.6)
        ax.add_patch(inflated_patch)
    # Original obstacle (inner)
        original_patch = MplPolygon(list(original_obs.exterior.coords), closed=True, facecolor='dimgray', edgecolor='black', alpha=1.0)
        ax.add_patch(original_patch)

    ax.add_patch(Circle((start.x, start.y), radius, color='green', label='Start'))
    # draw start angle as an arrow with a length of 10
    ax.arrow(start.x, start.y, 10 * np.cos(np.radians(start_angle)), 10 * np.sin(np.radians(start_angle)),
             head_width=1, head_length=2, fc='green', ec='green', label='Start Angle')

    if vip:
        ax.add_patch(Circle((vip.x, vip.y), radius, color='magenta', label='VIP'))

    for i, pt in enumerate(safe_points):
        ax.add_patch(Circle((pt.x, pt.y), radius, color='orange', label='Safe Point' if i == 0 else None))
        #ax.text(pt[0]+radius, pt[1]+radius, f'S{i}', color='orange')

    if others:
        for i, pt in enumerate(others):
            ax.add_patch(Circle((pt.x, pt.y), radius, color='blue', label='Other Balls' if i == 0 else None))
            #ax.text(pt[0]+radius, pt[1]+radius, f'O{i}', color='blue')

    ax.add_patch(Circle((end.x, end.y), radius, color='red', label='End'))

    # Path
    if full_path:
        xs = [pt.x for pt in full_path]
        ys = [pt.y for pt in full_path]
        ax.plot(xs, ys, 'r-', linewidth=2, label='Planned path')
        # Print point type
        for i, pt in enumerate(full_path):
            ax.text(pt.x + radius, pt.y + radius, f'{pt.type.capitalize()}', color='black', fontsize=8)
            if pt.type == 'safe':
                # Draw a line between point and the point.target
                ax.plot([pt.x, pt.target.x], [pt.y, pt.target.y], 'g--', linewidth=2, color='green')

    # Title
    order_text = "Visit order: Start"
    offset = 2 if has_vip else 1
    for idx in best_order:
        if has_vip and idx == 1:
            order_text += " → VIP"
        else:
            order_text += f" → O{idx - offset}"
    order_text += " → End"

    ax.set_title(order_text)
    ax.legend()
    ax.set_aspect('equal')
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.grid(True)

    if debug:
        plt.show()
    else:
        fig.canvas.draw()
        fig.canvas.flush_events()


