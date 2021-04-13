# Gerador de relatórios de uma simulação de tráfego
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 26 de Março de 2020

# Imports gerais de bibliotecas padrão
from os import listdir
import sys
from statistics import mean
from os.path import isdir, join
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from typing import Dict, List
from crossing_report import Reporter
from rich.console import Console
console = Console()


class GroupReporter:
    """
    Classe responsável por gerar os reports de uma simulação.
    """
    def __init__(self,
                 result_dir: str,
                 reporters: List[Reporter]):

        self.result_dir = result_dir
        self.reporters = reporters

    def make_split_plot(self):
        """
        """
        # Extrai as variáveis necessárias a partir dos DataFrames
        # Os ciclos considerados são dados pela interseção dos ciclos
        # executados em todos os testes.
        cs = set(list(range(1, 100)))
        for r in self.reporters:
            r_cycles = set(list(r.cycle_data["cycle_id"]))
            cs.intersection_update(r_cycles)
        cycles = list(cs)
        # Os estágios são os mesmos, pois os planos são os mesmos.
        # Logo, pega-se os estágios de qualquer um
        stages = list(range(len(self.reporters[0].cycle_data.columns) - 3))
        labels_estagios = [f"Estágio {s + 1}" for s in stages]
        arestas_estagios = [["W_C", "E_C"], ["N_C", "S_C"]]
        # As eids são as mesmas para todos
        eids = list(set(list(self.reporters[0].traffic_data["edge_id"])))
        # Prepara as variáveis para serem plotadas
        splits: List[Dict[int, Dict[int, float]]] = []
        for r in self.reporters:
            rs: Dict[int, Dict[int, float]] = {c: {s: 0. for s in stages}
                                               for c in cycles}
            for _, line in r.cycle_data.iterrows():
                c = line["cycle_id"]
                if c in cycles:
                    for s in stages:
                        rs[c][s] = line[f"split_stg_{s}"]
            splits.append(rs)
        occupations: List[Dict[str, Dict[int, float]]] = []
        counts: List[Dict[str, Dict[int, int]]] = []
        speeds: List[Dict[str, Dict[int, float]]] = []
        for r in self.reporters:
            os: Dict[str, Dict[int,
                               float]] = {e: {c: 0. for c in cycles}
                                          for e in eids}
            cs: Dict[str, Dict[int,
                               int]] = {e: {c: 0 for c in cycles}
                                        for e in eids}
            ss: Dict[str, Dict[int,
                               float]] = {e: {c: 0. for c in cycles}
                                          for e in eids}
            for _, line in r.traffic_data.iterrows():
                e = line["edge_id"]
                c = line["cycle_id"]
                if c in cycles:
                    os[e][c] = line["occupation"]
                    cs[e][c] = line["vehicle_count"]
                    ss[e][c] = line["average_speed"]
            occupations.append(os)
            counts.append(cs)
            speeds.append(ss)

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
            for j, split in enumerate(splits):
                legendar = j == 0 
                fig.add_trace(go.Scatter(
                    x=cycles,
                    y=[split[c][s] for c in cycles],
                    name=labels_estagios[i],
                    connectgaps=True,
                    showlegend=legendar,
                    line={"color": cores[i]}),
                row=1, col=1)
        for i, e in enumerate(labels_estagios):
            for occupation in occupations:
                fig.add_trace(go.Scatter(
                    x=cycles,
                    y=[max([occupation[eid][c]
                            for eid in arestas_estagios[i]])
                    for c in cycles],
                    name=f"{e}",
                    connectgaps=True,
                    showlegend=False,
                    line={"color": cores[i]}),
                row=2, col=1)
        labels_vias = [r"$\bar{q}_W$", r"$\bar{q}_N$", r"$\bar{q}_E$", r"$\bar{q}_S$"]
        for i, e in enumerate(["W_C", "N_C", "E_C", "S_C"]):
            for j, count in enumerate(counts):
                legendar = j == 0
                fig.add_trace(go.Scatter(
                    x=cycles,
                    y=[count[e][c] for c in cycles],
                    name=labels_vias[i],
                    connectgaps=True,
                    showlegend=legendar,
                    line={"color": cores[i % 2],
                        "dash": estilo[int(i / 2)]}),
                row=3, col=1)
        for i, e in enumerate(labels_estagios):
            for speed in speeds:
                fig.add_trace(go.Scatter(
                    x=cycles,
                    y=[mean([speed[eid][c]
                            for eid in arestas_estagios[i]])
                    for c in cycles],
                    name=f"{e}",
                    connectgaps=True,
                    showlegend=False,
                    line={"color": cores[i]}),
                row=4, col=1)
        fig.write_image(join(self.result_dir, "split.png"))

    def make_cycle_plot(self):
        """
        """
        # Extrai as variáveis necessárias a partir dos DataFrames
        # Os ciclos considerados são dados pela interseção dos ciclos
        # executados em todos os testes.
        cs = set(list(range(1, 100)))
        for r in self.reporters:
            r_cycles = set(list(r.cycle_data["cycle_id"]))
            print(f"{r.result_dir} - {r_cycles}")
            cs.intersection_update(r_cycles)
        cycles = list(cs)
        print(cycles)
        # Os estágios são os mesmos, pois os planos são os mesmos.
        # Logo, pega-se os estágios de qualquer um
        stages = list(range(len(self.reporters[0].cycle_data.columns) - 3))
        labels_estagios = [f"Estágio {s + 1}" for s in stages]
        arestas_estagios = [["W_C", "E_C"], ["N_C", "S_C"]]
        # As eids são as mesmas para todos
        eids = list(set(list(self.reporters[0].traffic_data["edge_id"])))
        # Prepara as variáveis para serem plotadas
        c_lengths: List[Dict[int, Dict[int, float]]] = []
        for r in self.reporters:
            c_l: Dict[int, Dict[int, float]] = {c: 0
                                                for c in cycles}
            for _, line in r.cycle_data.iterrows():
                c = line["cycle_id"]
                if c in cycles:
                    c_l[c] = int(round(line["end_time"][stages[-1]]
                                       - line["start_time"][stages[0]]))
            c_lengths.append(c_l)
        occupations: List[Dict[str, Dict[int, float]]] = []
        counts: List[Dict[str, Dict[int, int]]] = []
        speeds: List[Dict[str, Dict[int, float]]] = []
        for r in self.reporters:
            os: Dict[str, Dict[int,
                               float]] = {e: {c: 0. for c in cycles}
                                          for e in eids}
            cs: Dict[str, Dict[int,
                               int]] = {e: {c: 0 for c in cycles}
                                        for e in eids}
            ss: Dict[str, Dict[int,
                               float]] = {e: {c: 0. for c in cycles}
                                          for e in eids}
            for _, line in r.traffic_data.iterrows():
                e = line["edge_id"]
                c = line["cycle_id"]
                if c in cycles:
                    os[e][c] = line["occupation"]
                    cs[e][c] = line["vehicle_count"]
                    ss[e][c] = line["average_speed"]
            occupations.append(os)
            counts.append(cs)
            speeds.append(ss)

        # Cria o plot de dados
        cycle_ticks = [1, 12, 14, 24, 26, 34, 36, 42, 44, 52]
        fig = make_subplots(rows=4, cols=1,
                            shared_xaxes=True,
                            vertical_spacing=0.090,
                            subplot_titles=(r"$\text{Cycle length} \, (C)$",
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
                         ticktext=["1", "12", "14", "24", "26", "34",
                                   "36", "42", "44", "52"],
                         tickvals=cycle_ticks,
                         row=4, col=1)
        fig.update_yaxes(title_text=r"$\phi \, (\%)$",
                         ticktext=["40", "50", "60", "70", "80"],
                         tickvals=[40, 50, 60, 70, 80],
                         range=[40, 80],
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
        for j, c_len in enumerate(c_lengths):
            legendar = j == 0 
            fig.add_trace(go.Scatter(
                x=cycles,
                y=[c_len[c] for c in cycles],
                name=r"$C$",
                connectgaps=True,
                showlegend=legendar,
                line={"color": "#9999ff"}),
            row=1, col=1)
        for i, e in enumerate(labels_estagios):
            for occupation in occupations:
                fig.add_trace(go.Scatter(
                    x=cycles,
                    y=[max([occupation[eid][c]
                            for eid in arestas_estagios[i]])
                    for c in cycles],
                    name=f"{e}",
                    connectgaps=True,
                    showlegend=False,
                    line={"color": cores[i]}),
                row=2, col=1)
        labels_vias = [r"$\bar{q}_W$", r"$\bar{q}_N$", r"$\bar{q}_E$", r"$\bar{q}_S$"]
        for i, e in enumerate(["W_C", "N_C", "E_C", "S_C"]):
            for j, count in enumerate(counts):
                legendar = j == 0
                fig.add_trace(go.Scatter(
                    x=cycles,
                    y=[count[e][c] for c in cycles],
                    name=labels_vias[i],
                    connectgaps=True,
                    showlegend=legendar,
                    line={"color": cores[i % 2],
                        "dash": estilo[int(i / 2)]}),
                row=3, col=1)
        for i, e in enumerate(labels_estagios):
            for speed in speeds:
                fig.add_trace(go.Scatter(
                    x=cycles,
                    y=[mean([speed[eid][c]
                            for eid in arestas_estagios[i]])
                    for c in cycles],
                    name=f"{e}",
                    connectgaps=True,
                    showlegend=False,
                    line={"color": cores[i]}),
                row=4, col=1)
        fig.write_image(join(self.result_dir, "cycle.png"))

    def make_travel_time_plot(self):
        """
        """
        # Junta os dados de todos os veículos em um único DF
        travel_times = self.group_vehicle_travel_times()
        # Plota o histograma
        fig = go.Figure()
        fig.add_trace(go.Histogram(x=travel_times,
                               histnorm='probability',
                               xbins={"start": 20,
                                      "end": 120,
                                      "size": 5}
                               ))
        fig.update_layout(font_family="Times",
                          template="simple_white")
        fig.update_xaxes(showline=True, mirror=True)
        fig.update_yaxes(showline=True, mirror=True)
        fig.update_xaxes(title_text=r"$\text{Vehicle Travel Time (s)}$",
                         range=[20, 120])
        fig.update_yaxes(ticktext=["0%", "5%", "10%", "15%", "20%"],
                         tickvals=[0.00, 0.05, 0.10, 0.15, 0.20],
                         range=[0, 0.20])
        fig.write_image(join(self.result_dir, "travel_time.png"))

    def make_stopped_time_plot(self):
        """
        """
        # Junta os dados de todos os veículos em um único DF
        stopped_times = self.group_vehicle_stopped_times()
        # Plota o histograma
        fig = go.Figure()
        fig.add_trace(go.Histogram(x=stopped_times,
                                   histnorm='probability',
                                   xbins={"start": 0,
                                          "end": 120,
                                          "size": 5}
                                   ))
        fig.update_layout(font_family="Times",
                          template="simple_white")
        fig.update_xaxes(showline=True, mirror=True)
        fig.update_yaxes(showline=True, mirror=True)
        fig.update_xaxes(title_text=r"$\text{Vehicle Stopped Time (s)}$",
                         range=[0, 120])
        fig.update_yaxes(ticktext=["0%", "10%", "20%", "30%", "40%", "50%"],
                         tickvals=[0.00, 0.10, 0.20, 0.30, 0.40, 0.50],
                         range=[0, 0.50])
        fig.write_image(join(self.result_dir, "stopped_time.png"))

    def group_vehicle_travel_times(self) -> List[float]:
        """
        """
        travel_times: List[float] = []
        for r in self.reporters:
            travel_times += list(r.vehicle_data["travel_time"])
        return travel_times

    def group_vehicle_stopped_times(self) -> List[float]:
        """
        """
        stopped_times: List[float] = []
        for r in self.reporters:
            stopped_times += list(r.vehicle_data["stopped_time"])
        return stopped_times

    def make_result_plots(self):
        """
        Transforma os dados obtidos do diretório em arquivos com gráficos.
        """
        self.make_split_plot()
        self.make_cycle_plot()
        self.make_travel_time_plot()
        self.make_stopped_time_plot()


if __name__ == "__main__":
    # Retorna imediatamente em caso de não haver diretório
    if len(sys.argv) < 2:
        print("Por favor forneça um diretório válido.")
        exit(1)
    # Se forem passados 3 diretórios, faz o histograma comparativo
    if len(sys.argv) == 4:
        dir1 = sys.argv[1]
        reporters1: List[Reporter] = []
        for dir in listdir(dir1):
            if isdir(join(dir1, dir)):
                console.log(dir)
                reporters1.append(Reporter(join(dir1, dir)))
        group_reporter1 = GroupReporter(dir1,
                                        reporters1)
        dir2 = sys.argv[2]
        reporters2: List[Reporter] = []
        for dir in listdir(dir2):
            if isdir(join(dir2, dir)):
                console.log(dir)
                reporters2.append(Reporter(join(dir2, dir)))
        group_reporter2 = GroupReporter(dir2,
                                        reporters2)
        dir3 = sys.argv[3]
        reporters3: List[Reporter] = []
        for dir in listdir(dir3):
            if isdir(join(dir3, dir)):
                console.log(dir)
                reporters3.append(Reporter(join(dir3, dir)))
        group_reporter3 = GroupReporter(dir3,
                                        reporters3)
        # Faz o histograma
        vehicles_1 = group_reporter1.group_vehicle_travel_times()
        vehicles_2 = group_reporter2.group_vehicle_travel_times()
        vehicles_3 = group_reporter3.group_vehicle_travel_times()
        fig = go.Figure()
        fig.add_trace(go.Histogram(x=vehicles_1,
                                   histnorm='probability',
                                   xbins={"start": 20,
                                           "end": 120,
                                           "size": 5},
                                   name=r"$\text{Split control}$",
                                   marker_color="#9999ff",
                                   ))
        fig.add_trace(go.Histogram(x=vehicles_2,
                                   histnorm='probability',
                                   xbins={"start": 20,
                                           "end": 120,
                                           "size": 5},
                                   name=r"$\text{Cycle control}$",
                                   marker_color="#cc6600",
                                   ))
        fig.add_trace(go.Histogram(x=vehicles_3,
                                   histnorm='probability',
                                   xbins={"start": 20,
                                           "end": 120,
                                           "size": 5},
                                   name=r"$\text{Fixed time}$",
                                   marker_color="#ff9966",
                                   ))
        fig.update_traces(opacity=0.35)
        fig.update_layout(barmode='overlay')
        # Reduce opacity to see both histograms
        fig.update_layout(font_family="Times",
                          template="simple_white",
                          legend={"yanchor": "top",
                                  "xanchor": "left",
                                  "y": 0.95,
                                  "x": 0.75})
        fig.update_xaxes(showline=True, mirror=True)
        fig.update_yaxes(showline=True, mirror=True)
        fig.update_xaxes(title_text=r"$\text{Vehicle Travel Time (s)}$",
                         range=[20, 120])
        fig.update_yaxes(ticktext=["0%", "5%", "10%", "15%", "20%"],
                         tickvals=[0.00, 0.05, 0.10, 0.15, 0.20],
                         range=[0, 0.20])
        result_dir = "results/crossing"
        fig.write_image(join(result_dir, "compare_travel_times.png"))

    # Processa normalmente
    else:
        result_dir = sys.argv[1]
        console.log(f"Gerando DFs para {result_dir}")
        reporters: List[Reporter] = []
        for dir in listdir(result_dir):
            if isdir(join(result_dir, dir)):
                console.log(dir)
                reporters.append(Reporter(join(result_dir, dir)))
        group_reporter = GroupReporter(result_dir,
                                    reporters)
        group_reporter.make_result_plots()
