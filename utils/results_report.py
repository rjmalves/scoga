# Gerador de relatórios de uma simulação de tráfego
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 26 de Março de 2020

# Imports gerais de bibliotecas padrão
import sys
import pickle
from os.path import isfile, join
from pandas import DataFrame  # type: ignore
import plotly.express as px
# Disable the orca response timeout.
import plotly.io._orca
import retrying
unwrapped = plotly.io._orca.request_image_with_retrying.__wrapped__
wrapped = retrying.retry(wait_random_min=1000)(unwrapped)
plotly.io._orca.request_image_with_retrying = wrapped


class Reporter:
    """
    Classe responsável por gerar os reports de uma simulação.
    """
    def __init__(self, result_dir: str):
        self.result_dir = result_dir

    def process_tl_data(self) -> DataFrame:
        """
        Processa os dados de semáforos existentes no diretório da simulação.
        """
        tl_data: DataFrame = DataFrame()
        # Abre o arquivo com dados de semáforos
        full_path = join(result_dir, "trafficlights.pickle")
        if isfile(full_path):
            with open(full_path, "rb") as fileref:
                tl_data = pickle.load(fileref)
        # Adiciona a coluna 'y' = 1.0 só para plotar
        tl_data['y'] = 1.0
        # Retorna os dados de semáforos
        return tl_data

    def process_detector_data(self) -> DataFrame:
        """
        Processa os dados de detectores existentes no diretório da simulação.
        """
        det_data: DataFrame = DataFrame()
        # Abre o arquivo com dados de detectores
        full_path = join(result_dir, "detectors.pickle")
        if isfile(full_path):
            with open(full_path, "rb") as fileref:
                det_data = pickle.load(fileref)
        # Retorna os dados de detectores
        return det_data

    def process_node_data(self) -> DataFrame:
        """
        Processa os dados de nós existentes no diretório da simulação.
        """
        node_data: DataFrame = DataFrame()
        # Abre o arquivo com dados de interseções
        full_path = join(result_dir, "nodes.pickle")
        if isfile(full_path):
            with open(full_path, "rb") as fileref:
                node_data = pickle.load(fileref)
        # Adiciona a coluna 'y' = 1.0 só para plotar
        node_data['y'] = 1.0
        # Retorna os dados de interseções
        return node_data

    def process_edge_data(self) -> DataFrame:
        """
        Processa os dados de arestas existentes no diretório da simulação.
        """
        edge_data: DataFrame = DataFrame()
        # Abre o arquivo com dados de arestas
        full_path = join(result_dir, "edges.pickle")
        if isfile(full_path):
            with open(full_path, "rb") as fileref:
                edge_data = pickle.load(fileref)
        # Retorna os dados de arestas
        return edge_data

    def process_lane_data(self) -> DataFrame:
        """
        Processa os dados de faixas existentes no diretório da simulação.
        """
        lane_data: DataFrame = DataFrame()
        # Abre o arquivo com dados de arestas
        full_path = join(result_dir, "lanes.pickle")
        if isfile(full_path):
            with open(full_path, "rb") as fileref:
                lane_data = pickle.load(fileref)
        # Retorna os dados de arestas
        return lane_data

    def make_detector_plots(self):
        """
        """
        # Processa os dados de detectores
        det_data = self.process_detector_data()
        fig = px.line(det_data,
                      x='sampling_time',  # instantes de tempo
                      y='state',  # curvas de detecção
                      labels={'sampling_time': '',
                              'state': ''},  # substitui os nomes
                      template='none',  # template minimalista
                      range_y=[0, 1],  # deixa o eixo y preenchido (até 1.0)
                      facet_col='det_id',  # gera subplots segundo o semáforo
                      facet_col_wrap=3,  # salta de linha após 1 na coluna
                      title='',  # título geral
                      width=640,  # dimensões da imagem
                      height=480  # dimensões da imagem
                      )
        fig.update_traces(line={'width': 2})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showticklabels': False,
                          'showgrid': False})
        figname = join(self.result_dir, "") + "detectors.png"
        fig.write_image(figname)

    def make_tl_plots(self):
        """
        """
        # Processa os dados de semáforos
        tl_data = self.process_tl_data()
        fig = px.area(tl_data,
                      x='sampling_time',  # instantes de tempo
                      y='y',  # 1.0 constante só para ter a área
                      color='state',  # cor segundo o estágio
                      color_discrete_sequence=['#EF350D',  # vermelho
                                               '#0DEF85',  # verde
                                               '#F4CF38'],  # amarelo
                      labels={'sampling_time': '',
                              'y': '',
                              'state': 'Estado'},  # substitui os nomes
                      template='none',  # template minimalista
                      range_y=[0, 1],  # deixa o eixo y preenchido (até 1.0)
                      facet_col='tl_id',  # gera subplots segundo o semáforo
                      facet_col_wrap=3,  # salta de linha após 1 na coluna
                      title='',  # título geral
                      width=1920,  # dimensões da imagem
                      height=1080  # dimensões da imagem
                      )
        fig.update_traces(line={'width': 0})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showgrid': False})
        figname = join(self.result_dir, "") + "trafficlights.png"
        fig.write_image(figname)

    def make_node_plots(self):
        """
        """
        # Processa os dados dos nós
        node_data = self.process_node_data()
        self.make_node_cycle_plot(node_data)
        self.make_node_stage_plot(node_data)
        self.make_node_interval_plot(node_data)

    def make_node_cycle_plot(self, node_data: DataFrame):
        """
        """
        # Cria os plots
        fig = px.area(node_data,
                      x='sampling_time',  # instantes de tempo
                      y='y',  # 1.0 constante só para ter a área
                      color='cycle',  # cor segundo o estado
                      color_discrete_sequence=['#588da8',  # E1
                                               '#ccafaf',  # E2
                                               '#e58a8a'],  # E3
                      labels={'sampling_time': '',
                              'y': '',
                              'cycle': 'Ciclo'},  # substitui os nomes
                      template='none',  # template minimalista
                      range_y=[0, 1],  # deixa o eixo y preenchido (até 1.0)
                      facet_col='node_id',  # gera subplots segundo o semáforo
                      facet_col_wrap=3,  # salta de linha após 1 na coluna
                      title='',  # título geral
                      width=1920,  # dimensões da imagem
                      height=1080  # dimensões da imagem
                      )
        fig.update_traces(line={'width': 0})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showticklabels': False,
                          'showgrid': False})
        figname = join(self.result_dir, "") + "nodes_cycle.png"
        fig.write_image(figname)

    def make_node_stage_plot(self, node_data: DataFrame):
        """
        """
        # Cria os plots
        fig = px.area(node_data,
                      x='sampling_time',  # instantes de tempo
                      y='y',  # 1.0 constante só para ter a área
                      color='stage',  # cor segundo o estado
                      color_discrete_sequence=['#588da8',  # E1
                                               '#ccafaf',  # E2
                                               '#e58a8a'],  # E3
                      labels={'sampling_time': '',
                              'y': '',
                              'stage': 'Estágio'},  # substitui os nomes
                      template='none',  # template minimalista
                      range_y=[0, 1],  # deixa o eixo y preenchido (até 1.0)
                      facet_col='node_id',  # gera subplots segundo o semáforo
                      facet_col_wrap=3,  # salta de linha após 1 na coluna
                      title='',  # título geral
                      width=1920,  # dimensões da imagem
                      height=1080  # dimensões da imagem
                      )
        fig.update_traces(line={'width': 0})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showticklabels': False,
                          'showgrid': False})
        figname = join(self.result_dir, "") + "nodes_stage.png"
        fig.write_image(figname)

    def make_node_interval_plot(self, node_data: DataFrame):
        """
        """
        # Cria os plots
        fig = px.area(node_data,
                      x='sampling_time',  # instantes de tempo
                      y='y',  # 1.0 constante só para ter a área
                      color='interval',  # cor segundo o estado
                      color_discrete_sequence=['#588da8',  # E1
                                               '#ccafaf',  # E2
                                               '#e58a8a'],  # E3
                      labels={'sampling_time': '',
                              'y': '',
                              'interval': 'Intervalo'},  # substitui os nomes
                      template='none',  # template minimalista
                      range_y=[0, 1],  # deixa o eixo y preenchido (até 1.0)
                      facet_col='node_id',  # gera subplots segundo o semáforo
                      facet_col_wrap=3,  # salta de linha após 1 na coluna
                      title='',  # título geral
                      width=1920,  # dimensões da imagem
                      height=1080  # dimensões da imagem
                      )
        fig.update_traces(line={'width': 0})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showticklabels': False,
                          'showgrid': False})
        figname = join(self.result_dir, "") + "nodes_interval.png"
        fig.write_image(figname)

    def make_edge_plots(self):
        """
        """
        # Processa os dados das arestas
        edge_data = self.process_edge_data()
        # Cria o plot de dados da aresta
        self.make_edge_averagespeed_plot(edge_data)
        self.make_edge_vehiclecount_plot(edge_data)
        self.make_edge_occupancy_plot(edge_data)

    def make_edge_averagespeed_plot(self, df: DataFrame):
        """
        """
        fig = px.area(df,
                      x='sampling_time',
                      y='average_speed',
                      labels={'sampling_time': '',
                              'average_speed': ''},
                      template='none',
                      facet_col='edge_id',
                      facet_col_wrap=4,
                      title='',  # título geral
                      width=1920,  # dimensões da imagem
                      height=1080  # dimensões da imagem
                      )
        fig.update_traces(line={'width': 2})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showticklabels': False,
                          'showgrid': False})
        fig.write_image(join(self.result_dir, "edges_averagespeed.png"))

    def make_edge_vehiclecount_plot(self, df: DataFrame):
        """
        """
        fig = px.area(df,
                      x='sampling_time',
                      y='vehicle_count',
                      labels={'sampling_time': '',
                              'vehicle_count': ''},
                      template='none',
                      facet_col='edge_id',
                      facet_col_wrap=4,
                      title='',  # título geral
                      width=1920,  # dimensões da imagem
                      height=1080  # dimensões da imagem
                      )
        fig.update_traces(line={'width': 2})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showticklabels': False,
                          'showgrid': False})
        fig.write_image(join(self.result_dir, "edges_vehiclecount.png"))

    def make_edge_occupancy_plot(self, df: DataFrame):
        """
        """
        fig = px.area(df,
                      x='sampling_time',
                      y='average_occupancy',
                      labels={'sampling_time': '',
                              'average_occupancy': ''},
                      template='none',
                      facet_col='edge_id',
                      facet_col_wrap=4,
                      title='',  # título geral
                      width=1920,  # dimensões da imagem
                      height=1080  # dimensões da imagem
                      )
        fig.update_traces(line={'width': 2})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showticklabels': False,
                          'showgrid': False})
        fig.write_image(join(self.result_dir, "edges_occupancy.png"))

    def make_lane_plots(self):
        """
        """
        # Processa os dados das faixas
        lane_data = self.process_lane_data()
        # Cria o plot de dados da faixa
        self.make_lane_averagespeed_plot(lane_data)
        self.make_lane_vehiclecount_plot(lane_data)
        self.make_lane_occupancy_plot(lane_data)

    def make_lane_averagespeed_plot(self, df: DataFrame):
        """
        """
        fig = px.area(df,
                      x='sampling_time',
                      y='average_speed',
                      labels={'sampling_time': '',
                              'average_speed': ''},
                      template='none',
                      facet_col='lane_id',
                      facet_col_wrap=4,
                      title='',  # título geral
                      width=1920,  # dimensões da imagem
                      height=1080  # dimensões da imagem
                      )
        fig.update_traces(line={'width': 2})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showticklabels': False,
                          'showgrid': False})
        fig.write_image(join(self.result_dir, "lanes_averagespeed.png"))

    def make_lane_vehiclecount_plot(self, df: DataFrame):
        """
        """
        fig = px.area(df,
                      x='sampling_time',
                      y='vehicle_count',
                      labels={'sampling_time': '',
                              'vehicle_count': ''},
                      template='none',
                      facet_col='lane_id',
                      facet_col_wrap=4,
                      title='',  # título geral
                      width=1920,  # dimensões da imagem
                      height=1080  # dimensões da imagem
                      )
        fig.update_traces(line={'width': 2})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showticklabels': False,
                          'showgrid': False})
        fig.write_image(join(self.result_dir, "lanes_vehiclecount.png"))

    def make_lane_occupancy_plot(self, df: DataFrame):
        """
        """
        fig = px.area(df,
                      x='sampling_time',
                      y='average_occupancy',
                      labels={'sampling_time': '',
                              'average_occupancy': ''},
                      template='none',
                      facet_col='lane_id',
                      facet_col_wrap=4,
                      title='',  # título geral
                      width=1920,  # dimensões da imagem
                      height=1080  # dimensões da imagem
                      )
        fig.update_traces(line={'width': 2})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showticklabels': False,
                          'showgrid': False})
        fig.write_image(join(self.result_dir, "lanes_occupancy.png"))

    def make_result_plots(self):
        """
        Transforma os dados obtidos do diretório em arquivos com gráficos.
        """
        # self.make_detector_plots()
        self.make_tl_plots()
        self.make_node_plots()
        self.make_edge_plots()
        # self.make_lane_plots()


if __name__ == "__main__":
    # Retorna imediatamente em caso de não haver diretório
    if len(sys.argv) < 2:
        print("Por favor forneça um diretório válido.")
        exit(1)
    # Processa normalmente
    result_dir = sys.argv[1]
    reporter = Reporter(result_dir)
    reporter.make_result_plots()
