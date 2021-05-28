from matplotlib import pyplot as plt
import numpy as np

def autolabel(ax, rects):
    for rect in rects:
        height = rect.get_height()
        ax.text(rect.get_x() + rect.get_width() / 2., (1.02 * height), '%.5f' % height, ha='center', va='bottom')

def generate_diference_between_kinematics_computation(x_labels, forward_kinematic, inverse_kinematic):
    x = np.arange(len(x_labels))  # the label locations
    bar_width = 0.3

    fig, ax = plt.subplots()
    rects1 = ax.bar(x - (bar_width / 2), forward_kinematic, bar_width, label='Cinematica direta', color='orange')
    rects2 = ax.bar(x + (bar_width / 2), inverse_kinematic, bar_width, label='Cinematica inversa', color='steelblue')

    ax.set_title('Maximos erros dos calculos de cinematica direta e inversa')
    ax.set_xticks(x)
    ax.set_xticklabels(x_labels)
    ax.set_xlabel('Conjunto de valores de juntas')
    ax.set_ylabel('Erro [m ou rad]')
    ax.legend()

    autolabel(ax, rects1)
    autolabel(ax, rects2)

    fig.tight_layout()

def generate_translation_errors(x_labels, translation_errors):
    x = np.arange(len(x_labels))  # the label locations
    
    fig, ax = plt.subplots()

    ax.plot(x_labels, translation_errors, marker='o')

    ax.set_title('Erros de translacao')
    ax.set_xticks(x)
    ax.set_xticklabels(x_labels)
    ax.set_xlabel('Conjunto de valores de juntas')
    ax.set_ylabel('Erro [m]')
    ax.legend(['Eixo X', 'Eixo Y', 'Eixo Z'])
    ax.grid(alpha=0.2)
    # ax.set_xlim([0, len(x) - 1])
    
    fig.tight_layout()

def generate_joints_errors(x_labels, joints_errors):
    x = np.arange(len(x_labels))  # the label locations
    
    fig, ax = plt.subplots()

    ax.plot(x_labels, joints_errors, marker='o')
    
    ax.set_title('Erros do angulo das juntas')
    ax.set_xticks(x)
    ax.set_xticklabels(x_labels)
    ax.set_xlabel('Conjunto de valores de juntas')
    ax.set_ylabel('Erro [rad]')
    ax.legend(['junta 1', 'junta 2', 'junta 3', 'junta 4', 'junta 5', 'junta 6'])
    ax.grid(alpha=0.2)
    # ax.set_xlim([0, len(x) - 1])
    
    fig.tight_layout()

def show_charts():
    plt.show()
