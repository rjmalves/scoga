# Gerador de relatórios de uma simulação de tráfego
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 26 de Março de 2020

# Imports gerais de bibliotecas padrão
from os import curdir
from platform import node
import sys
from statistics import mean
import pickle
from os.path import isfile, join
from pandas import DataFrame  # type: ignore
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from typing import Dict, Tuple, List
from rich.console import Console
console = Console()
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
        self.node_data = self.process_node_data()
        self.edge_data = self.process_edge_data()
        self.vehicle_data = self.process_vehicle_data()
        self.cycle_data = self.process_cycle_data()
        self.traffic_data = self.process_traffic_data()

    def process_node_data(self) -> DataFrame:
        """
        Processa os dados de nós existentes no diretório da simulação.
        """
        node_data: DataFrame = DataFrame()
        # Abre o arquivo com dados de interseções
        full_path = join(self.result_dir, "nodes.pickle")
        if isfile(full_path):
            with open(full_path, "rb") as fileref:
                node_data = pickle.load(fileref)
        # Retorna os dados de interseções
        return node_data

    def process_edge_data(self) -> DataFrame:
        """
        Processa os dados de arestas existentes no diretório da simulação.
        """
        edge_data: DataFrame = DataFrame()
        # Abre o arquivo com dados de arestas
        full_path = join(self.result_dir, "edges.pickle")
        if isfile(full_path):
            with open(full_path, "rb") as fileref:
                edge_data = pickle.load(fileref)
        # Retorna os dados de arestas
        return edge_data
    
    def process_vehicle_data(self) -> DataFrame:
        """
        Processa os dados de veículos existentes no diretório da simulação.
        """
        vehicle_data: DataFrame = DataFrame()
        # Abre o arquivo com dados de arestas
        full_path = join(self.result_dir, "vehicles.pickle")
        if isfile(full_path):
            with open(full_path, "rb") as fileref:
                vehicle_data = pickle.load(fileref)
        # Retorna os dados de arestas
        return vehicle_data

    def split_by_cycle(self,    
                       df: DataFrame) -> Tuple[Dict[int, float],
                                               Dict[int, float],
                                               Dict[int, 
                                                    Dict[int,
                                                         float]]]:
        """
        """
        # Obtém os ciclos que aconteceram e ignora o primeiro
        # e o último (porque não são completos)
        cycles = list(set(list(df["cycle"])))[1:-1]
        last_cycle = max(cycles) + 1
        # Obtém os estágios que existiram
        stages = list(set(list(df["stage"])))
        # Cria a variável que irá armazenar os splits
        start_time = {c: {s: 0. for s in stages}
                      for c in cycles}
        end_time = {c: {s: 0. for s in stages}
                    for c in cycles}
        splits = {c: {s: 0. for s in stages}
                  for c in cycles}
        # Varre no df, para cada um dos ciclos, e guarda
        # o instante de início e fim dos estágios.
        current_cycle = 0
        current_stage = 0
        for _, line in df.iterrows():
            c = line["cycle"]
            s = line["stage"]
            t = line["sampling_time"]
            if c == 0:
                continue
            if c == last_cycle:
                break
            if c != current_cycle:
                start_time[c][s] = max([t - .1, 0])
                current_cycle = c
            if s != current_stage:
                start_time[c][s] = max([t - .1, 0])
                current_stage = s
            end_time[c][s] = t
        # Calcula os splits
        for c in cycles:
            dur = (max([end_time[c][s] for s in stages]) -
                   min([start_time[c][s] for s in stages]))
            for s in stages:
                splits[c][s] = (end_time[c][s] - start_time[c][s]) / dur
                if splits[c][s] != 0.5:
                    print(f"c = {c} s = {s} split = {splits[c][s]}")

        return start_time, end_time, splits

    def traffic_data_by_cycle(self,
                              df: DataFrame,
                              start_time: Dict[int, float],
                              end_time: Dict[int, float]
                              ) -> Tuple[Dict[str, Dict[int, float]],
                                         Dict[str, Dict[int, float]],
                                         Dict[str, Dict[int, float]]]:
        """
        """
        # Extrai as arestas possíveis
        eids = list(set(list(df["edge_id"])))
        # Extrai os ciclos possíveis e os estágios
        cycles = list(start_time.keys())
        stages = list(start_time[cycles[0]].keys())
        fs = stages[0]
        ls = stages[-1]
        occ = {e: {c: 0.
                   for c in cycles}
               for e in eids}
        veh_c = {e: {c: 0.
                     for c in cycles}
                 for e in eids}
        avg_spd = {e: {c: 0.
                       for c in cycles}
                   for e in eids}
        last_veh_c = {e: 0 for e in eids}
        for _, line in df.iterrows():
            t = line["sampling_time"]
            avg_speed = line["average_speed"]
            veh_count = line["vehicle_count"]
            avg_occ = line["average_occupancy"]
            eid = line["edge_id"]
            for c in cycles:
                if (t >= start_time[c][fs] and
                        t < end_time[c][ls]):
                    occ[eid][c] += avg_occ
                    # Pega somente valores positivos (fluxo de entrada)
                    vc = max([0,
                              veh_count - last_veh_c[eid]])
                    veh_c[eid][c] += vc
                    avg_spd[eid][c] += avg_speed
                    # Guarda o último valor de veh_c
                    last_veh_c[eid] = veh_count
                    break
        # Realiza a divisão para gerar em %
        for c in cycles:
            # Calcula o número de amostras
            n_samples = (end_time[c][ls] -
                            start_time[c][fs]) / 0.1
            for e in eids:
                occ[e][c] /= n_samples
                avg_spd[e][c] /= n_samples

        return occ, veh_c, avg_spd

    def process_cycle_data(self):
        """
        """
        cycle_data = DataFrame()
        full_path = join(self.result_dir, "cycles.pickle")
        if isfile(full_path):
            with open(full_path, "rb") as fileref:
                cycle_data = pickle.load(fileref)
        else:
            start, end, splits = self.split_by_cycle(self.node_data)
            cycle_ids = list(start.keys())
            cycle_data = DataFrame()
            cycle_data["cycle_id"] = cycle_ids
            cycle_data["start_time"] = [start[c] for c in cycle_ids]
            cycle_data["end_time"] = [end[c] for c in cycle_ids]
            for s in splits[1].keys():
                cycle_data[f"split_stg_{s}"] = [splits[c][s]
                                                  for c in cycle_ids]
            # Salva para não ter que calcular da próxima vez
            with open(full_path, "wb") as f:
                pickle.dump(cycle_data, f)

        return cycle_data

    def process_traffic_data(self):
        """
        """
        traffic_data = DataFrame()
        full_path = join(self.result_dir, "traffic.pickle")
        if isfile(full_path):
            with open(full_path, "rb") as fileref:
                traffic_data = pickle.load(fileref)
        else:
            cycle_ids = list(self.cycle_data["cycle_id"])
            s_t = list(self.cycle_data["start_time"])
            e_t = list(self.cycle_data["end_time"])
            start = {c: s for c, s in zip(cycle_ids, s_t)}
            end = {c: e for c, e in zip(cycle_ids, e_t)}
            occ, cts, spd = self.traffic_data_by_cycle(self.edge_data,
                                                       start,
                                                       end)
            # Prepara as colunas que irão criar o DataFrame
            eids: List[str] = []
            cycles: List[int] = []
            occs: List[float] = []
            counts: List[int] = []
            speeds: List[float] = []
            for e in list(occ.keys()):
                for c in cycle_ids:
                    eids.append(e)
                    cycles.append(c)
                    occs.append(occ[e][c])
                    counts.append(cts[e][c])
                    speeds.append(spd[e][c])
            traffic_data = DataFrame()
            traffic_data["edge_id"] = eids
            traffic_data["cycle_id"] = cycles
            traffic_data["occupation"] = occs
            traffic_data["vehicle_count"] = counts
            traffic_data["average_speed"] = speeds
            # Salva para não ter que calcular da próxima vez
            with open(full_path, "wb") as f:
                pickle.dump(traffic_data, f)

        return traffic_data

    def make_cycle_plot(self):
        """
        """
        # Extrai as variáveis necessárias a partir dos DataFrames
        cycles = list(set(list(self.cycle_data["cycle_id"])))
        stages = list(range(len(self.cycle_data.columns) - 3))
        labels_estagios = [f"Estágio {s + 1}" for s in stages]
        arestas_estagios = [["W_C", "E_C"], ["N_C", "S_C"]]
        splits: Dict[int, Dict[int, float]] = {c: {s: 0. for s in stages}
                                               for c in cycles}
        for _, line in self.cycle_data.iterrows():
            c = line["cycle_id"]
            for s in stages:
                splits[c][s] = line[f"split_stg_{s}"]
        eids = list(set(list(self.traffic_data["edge_id"])))
        occupations: Dict[str, Dict[int,
                                    float]] = {e: {c: 0. for c in cycles}
                                               for e in eids}
        counts: Dict[str, Dict[int,
                               int]] = {e: {c: 0 for c in cycles}
                                        for e in eids}
        speeds: Dict[str, Dict[int,
                               float]] = {e: {c: 0. for c in cycles}
                                          for e in eids}
        for _, line in self.traffic_data.iterrows():
            e = line["edge_id"]
            c = line["cycle_id"]
            occupations[e][c] = line["occupation"]
            counts[e][c] = line["vehicle_count"]
            speeds[e][c] = line["average_speed"]
        # Cria o plot de dados
        cycle_ticks = [1, 10, 12, 20, 22, 30, 32, 40]
        fig = make_subplots(rows=4, cols=1,
                            shared_xaxes=True,
                            vertical_spacing=0.090,
                            subplot_titles=(r"$\text{Splits by stage} \, (\phi)$",
                                            r"$\text{Max. occupation by stage} \, (O)$",
                                            r"$\text{Average flow by lane} \, (\bar{q})$",
                                            r"$\text{Average speed by stage} \, (\bar{v})$"))
        fig.update_layout(font_family="Times",
                          template="simple_white",
                          legend={"orientation": "h",
                          "yanchor": "top",
                          "xanchor": "left",
                          "y": 1.15,
                          "x": 0})
        fig.update_xaxes(showline=True, mirror=True)
        fig.update_yaxes(showline=True, mirror=True)
        # Adiciona os nomes dos eixos
        fig.update_xaxes(tickvals=cycle_ticks,
                         row=1, col=1)
        fig.update_xaxes(tickvals=cycle_ticks,
                         row=2, col=1)
        fig.update_xaxes(tickvals=cycle_ticks,
                         row=3, col=1)
        fig.update_xaxes(title_text=r"$Cycle$",
                         ticktext=["1", "10", "12", "20", "22", "30", "32", "40"],
                         tickvals=cycle_ticks,
                         row=4, col=1)
        fig.update_yaxes(title_text=r"$\phi \, (\%)$",
                         ticktext=["20", "50", "80"],
                         tickvals=[0.2, 0.5, 0.8],
                         range=[0, 1],
                         row=1, col=1)
        fig.update_yaxes(title_text=r"$O \, (\%)$",
                         ticktext=["10", "40"],
                         tickvals=[0.1, 0.4],
                         row=2, col=1)
        fig.update_yaxes(title_text=r"$\bar{q} \, (veh/min)$",
                         range=[5, 20],
                         ticktext=["5", "20"],
                         tickvals=[5, 20],
                         row=3, col=1)
        fig.update_yaxes(title_text=r"$\bar{v} \, (m/s)$",
                         range=[2, 10],
                         ticktext=["2", "6", "10"],
                         tickvals=[2, 6, 10],
                         row=4, col=1)
        cores = ["#9999ff", "#ff9966"]
        estilo = ["dash", "dot"]
        labels_estagios = [r"$\phi_1, \, O_1, \, \bar{v}_1$",
                           r"$\phi_2, \, O_2, \, \bar{v}_2$"]
        for i, s in enumerate(stages):
            fig.add_trace(go.Scatter(
                x=cycles,
                y=[splits[c][s] for c in cycles],
                name=labels_estagios[i],
                connectgaps=True,
                line={"color": cores[i]}),
            row=1, col=1)
        for i, e in enumerate(labels_estagios):
            fig.add_trace(go.Scatter(
                x=cycles,
                y=[max([occupations[eid][c]
                        for eid in arestas_estagios[i]])
                   for c in cycles],
                name=f"{e}",
                connectgaps=True,
                showlegend=False,
                line={"color": cores[i]}),
            row=2, col=1)
        labels_vias = [r"$\bar{q}_W$", r"$\bar{q}_N$", r"$\bar{q}_E$", r"$\bar{q}_S$"]
        for i, e in enumerate(["W_C", "N_C", "E_C", "S_C"]):
            fig.add_trace(go.Scatter(
                x=cycles,
                y=[counts[e][c] for c in cycles],
                name=labels_vias[i],
                connectgaps=True,
                line={"color": cores[i % 2],
                      "dash": estilo[int(i / 2)]}),
            row=3, col=1)
        for i, e in enumerate(labels_estagios):
            fig.add_trace(go.Scatter(
                x=cycles,
                y=[mean([speeds[eid][c]
                         for eid in arestas_estagios[i]])
                   for c in cycles],
                name=f"{e}",
                connectgaps=True,
                showlegend=False,
                line={"color": cores[i]}),
            row=4, col=1)
        fig.write_image(join(self.result_dir, "teste_crossing.png"))

    def make_travel_time_plot(self):
        """
        """
        # Processa os dados dos veículos
        vehicle_data = self.process_vehicle_data()
        # Plota o histograma
        fig = px.histogram(vehicle_data, x="travel_time")
        fig.write_image(join(self.result_dir, "travel_time.png"))


    def make_result_plots(self):
        """
        Transforma os dados obtidos do diretório em arquivos com gráficos.
        """
        self.make_cycle_plot()
        self.make_travel_time_plot()

if __name__ == "__main__":
    # Retorna imediatamente em caso de não haver diretório
    if len(sys.argv) < 2:
        print("Por favor forneça um diretório válido.")
        exit(1)
    # Processa normalmente
    result_dir = sys.argv[1]
    console.log(f"Gerando DFs para {result_dir}")
    reporter = Reporter(result_dir)
    reporter.make_result_plots()
