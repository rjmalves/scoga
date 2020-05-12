# Gerador de relatórios de uma simulação de tráfego
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 26 de Março de 2020

# Imports gerais de bibliotecas padrão
import sys
import pickle
from os import listdir
from os.path import isfile, join
import matplotlib.pyplot as plt
from matplotlib import cm
from typing import Dict, Tuple, List
from pandas import DataFrame  # type: ignore
import plotly.express as px


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

    def process_detector_data(self) -> Dict[str,
                                            Tuple[List[float], List[int]]]:
        """
        Processa os dados de detectores existentes no diretório da simulação.
        """
        detector_data_dict: Dict[str, Tuple[List[float], List[int]]] = {}
        # Para cada arquivo no diretório
        for f in listdir(result_dir):
            full_path = join(result_dir, f)
            if isfile(full_path):
                # Se o arquivo é de detector, processa os dados
                if "detector" in f and "pickle" in f:
                    with open(full_path, "rb") as fileref:
                        data = pickle.load(fileref)
                        det_name = f.split(".")[0]
                        times = [d[0] for d in data]
                        states = [d[1] for d in data]
                        detector_data_dict[det_name] = (times, states)
        # Retorna os dados de detector
        return detector_data_dict

    def process_node_data(self) -> Dict[str, Tuple[List[float], List[int]]]:
        """
        Processa os dados de nós existentes no diretório da simulação.
        """
        node_data_dict: Dict[str, Tuple[List[float], List[int]]] = {}
        # Para cada arquivo no diretório
        for f in listdir(result_dir):
            full_path = join(result_dir, f)
            if isfile(full_path):
                # Se o arquivo é de semáforo, processa os dados
                if "node" in f and "pickle" in f:
                    with open(full_path, "rb") as fileref:
                        data = pickle.load(fileref)
                        tl_name = f.split(".")[0]
                        times = [d[0] for d in data]
                        stages = [d[1] for d in data]
                        n_stg = max(stages) + 1
                        # Substitui os valores para plot
                        for i in range(n_stg):
                            stages = [1 + 1e-3 * (i + 1) if s == i else s
                                      for s in stages]
                        # Adiciona o elemento
                        node_data_dict[tl_name] = (times, stages)
        # Retorna os dados de semáforos
        return node_data_dict

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
        # Cria os subplots, compartilhando o eixo do tempo
        n_dets = len(det_data.keys())
        fig, axs = plt.subplots(n_dets, 1, sharex=True)
        axs[n_dets - 1].set_xlabel("Tempo (s)")
        for i, (det_name, data) in enumerate(det_data.items()):
            axs[i].plot(data[0], data[1])
            axs[i].set_title(det_name)
        figname = join(self.result_dir, "") + "detectors.pdf"
        plt.savefig(figname, orientation="portrait", papertype="a4")

    def make_tl_plots(self):
        """
        """
        # Processa os dados de semáforos
        tl_data = self.process_tl_data()
        fig = px.area(tl_data,
                      x='sampling_time',  # instantes de tempo
                      y='y',  # 1.0 constante só para ter a área
                      color='state',  # cor segundo o estado
                      color_discrete_sequence=['#EF350D',  # vermelho
                                               '#0DEF85',  # verde
                                               '#F4CF38'],  # amarelo
                      labels={'sampling_time': '',
                              'y': '',
                              'state': 'Estado'},  # substitui os nomes
                      template='none',  # template minimalista
                      range_y=[0, 1],  # deixa o eixo y preenchido (até 1.0)
                      facet_col='tl_id',  # gera subplots segundo o semáforo
                      facet_col_wrap=1,  # salta de linha após 1 na coluna
                      title='',  # título geral
                      width=640,  # dimensões da imagem
                      height=480  # dimensões da imagem
                      )
        fig.update_traces(line={'width': 0})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showgrid': False})
        figname = join(self.result_dir, "") + "trafficlights.pdf"
        fig.write_image(figname)

    def make_node_plots(self):
        """
        """
        # Processa os dados dos nós
        node_data = self.process_node_data()
        # Cria os plots
        n_inters = len(node_data.keys())
        fig, axs = plt.subplots(n_inters, 1, sharex=True)
        if n_inters > 1:
            axs[n_inters - 1].set_xlabel("Tempo (s)")
        else:
            axs.set_xlabel("Tempo (s)")
        for i, (inter_name, data) in enumerate(node_data.items()):
            if n_inters > 1:
                axs_tmp = axs[i]
            else:
                axs_tmp = axs
            axs_tmp.plot(data[0], data[1])
            # Força a escala do eixo y
            axs_tmp.set_ylim(bottom=0.0, top=1.0)
            # Define o colormap
            n_stg = len(set(data[1]))
            colors = cm.get_cmap('Set1', n_stg).colors
            # Faz um preenchimento com as cores
            for i in range(n_stg):
                axs_tmp.fill_between(data[0],
                                     0,
                                     data[1],
                                     where=[d == 1 + 1e-3 * (i + 1)
                                            for d in data[1]],
                                     facecolor=colors[i],
                                     interpolate=True)
            # Da um título
            axs_tmp.set_title(inter_name)
        figname = join(self.result_dir, "") + "nodes.pdf"
        plt.savefig(figname, orientation="portrait", papertype="a4")

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
                      facet_col_wrap=3
                      )
        fig.update_traces(line={'width': 2})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showticklabels': False,
                          'showgrid': False})
        fig.write_image(join(self.result_dir, "edges_averagespeed.pdf"))

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
                      facet_col_wrap=3
                      )
        fig.update_traces(line={'width': 2})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showticklabels': False,
                          'showgrid': False})
        fig.write_image(join(self.result_dir, "edges_vehiclecount.pdf"))

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
                      facet_col_wrap=3
                      )
        fig.update_traces(line={'width': 2})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showticklabels': False,
                          'showgrid': False})
        fig.write_image(join(self.result_dir, "edges_occupancy.pdf"))

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
                      facet_col_wrap=3
                      )
        fig.update_traces(line={'width': 2})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showticklabels': False,
                          'showgrid': False})
        fig.write_image(join(self.result_dir, "lanes_averagespeed.pdf"))

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
                      facet_col_wrap=3
                      )
        fig.update_traces(line={'width': 2})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showticklabels': False,
                          'showgrid': False})
        fig.write_image(join(self.result_dir, "lanes_vehiclecount.pdf"))

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
                      facet_col_wrap=3
                      )
        fig.update_traces(line={'width': 2})
        fig.update_yaxes({'showticklabels': False,
                          'showgrid': False})
        fig.update_xaxes({'zeroline': False,
                          'showticklabels': False,
                          'showgrid': False})
        fig.write_image(join(self.result_dir, "lanes_occupancy.pdf"))

    def make_result_plots(self):
        """
        Transforma os dados obtidos do diretório em arquivos com gráficos.
        """
        self.make_detector_plots()
        self.make_tl_plots()
        self.make_node_plots()
        self.make_edge_plots()
        self.make_lane_plots()


if __name__ == "__main__":
    # Retorna imediatamente em caso de não haver diretório
    if len(sys.argv) < 2:
        print("Por favor forneça um diretório válido.")
        exit(1)
    # Processa normalmente
    result_dir = sys.argv[1]
    reporter = Reporter(result_dir)
    reporter.make_result_plots()
