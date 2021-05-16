from matplotlib import pyplot as plt
import numpy as np

def autolabel(ax, rects):
    for rect in rects:
        height = rect.get_height()
        ax.text(rect.get_x() + rect.get_width() / 2., (1.02 * height), '%.3f' % height, ha='center', va='bottom')

def generate_diference_between_kinematics_computation(x_labels, forward_kinematic, inverse_kinematic):
    x = np.arange(len(x_labels))  # the label locations
    bar_width = 0.3

    fig, ax = plt.subplots()
    rects1 = ax.bar(x - (bar_width / 2), forward_kinematic, bar_width, label='Cinematica direta', color='orange')
    rects2 = ax.bar(x + (bar_width / 2), inverse_kinematic, bar_width, label='Cinematica inversa', color='steelblue')

    ax.set_xlabel('Conjunto de juntas')
    ax.set_ylabel('Erro [m]')
    ax.set_title('Maximos erros dos calculos de cinematica direta e inversa')
    ax.set_xticks(x)
    ax.set_xticklabels(x_labels)
    ax.legend()

    autolabel(ax, rects1)
    autolabel(ax, rects2)

    fig.tight_layout()

    plt.show()
