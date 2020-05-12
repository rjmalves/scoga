# Imports gerais de bibliotecas padrão
import pickle
from os import listdir
from os.path import isfile, join
from typing import Dict
from pandas import DataFrame
import plotly.express as px
from plotly.graph_objects import Figure, Bar
from plotly.subplots import make_subplots

def process_tl_data() -> Dict[str, DataFrame]:
    """
    Processa os dados de arestas existentes no diretório da simulação.
    """
    result_dir = 'results/cross/1589312585'
    tl_data = DataFrame()
    # Para cada arquivo no diretório
    for f in listdir(result_dir):
        full_path = join(result_dir, f)
        if isfile(full_path):
            # Se o arquivo é de aresta, processa os dados
            if f == "trafficlights.pickle":
                with open(full_path, "rb") as fileref:
                    tl_data = pickle.load(fileref)
    # Retorna os dados de arestas
    return tl_data


def process_tl_id_figure(df: DataFrame,
                         ) -> Figure:
    """
    """
    fig = px.area(df,
                  x='sampling_time',
                  y='y',
                  color='state',
                  color_discrete_sequence=['#EF350D',
                                           '#0DEF85',
                                           '#F4CF38'],
                  title='Grupo semafóricos',
                  labels={'sampling_time': 'Tempo (s)',
                          'y': '',
                          'state': 'Estado'},
                  template='none',
                  range_y=[0, 1],
                  facet_col='tl_id',
                  facet_col_wrap=1
                  )
    fig.update_traces(line={'width': 0})
    fig.update_yaxes({'showticklabels': False,
                      'showgrid': False})
    fig.update_xaxes({'zeroline': False,
                      'showgrid': False})
    return fig
    # fig.write_image("results/cross/1589312585/trafficlights.pdf")


if __name__ == "__main__":
    tl_data = process_tl_data()
    tl_data['y'] = 1.0
    fig = process_tl_id_figure(tl_data)
    fig.show()
