from deap import base
from deap import creator
from deap import tools

import random

creator.create("FitnessMax", base.Fitness, weights=(1.0,))
creator.create("Individual", list, fitness=creator.FitnessMax)

toolbox = base.Toolbox()

if __name__ == "__main__":

    # Attribute generator
    toolbox.register("attr_bool", random.randint, 0, 1)
    # Structure initializers
    toolbox.register("individual",
                     tools.initRepeat,
                     creator.Individual,
                     toolbox.attr_bool,
                     100)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    def evalOneMax(individual):
        return sum(individual),

    toolbox.register("evaluate", evalOneMax)
    toolbox.register("mate", tools.cxTwoPoint)
    toolbox.register("mutate", tools.mutFlipBit, indpb=0.05)
    toolbox.register("select", tools.selTournament, tournsize=3)

    # MAIN
    pop = toolbox.population(n=300)
    # Evaluate the entire population
    fitnesses = list(map(toolbox.evaluate, pop))
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit
    # CXPB é a probabilidade de crossover
    # MUTPB é a probabilidade de mutação
    CXPB, MUTPB = 0.5, 0.2
    # Extrai todas as fitnesses
    fits = [ind.fitness.values[0] for ind in pop]
    # Guarda o número de gerações
    g = 0
    # Começa a evolução
    while max(fits) < 100 and g < 1000:
        # Nova geração
        g = g + 1
        print("-- Generation %i --" % g)
        # Seleciona a próxima geração
        offspring = toolbox.select(pop, len(pop))
        # Clona os indivíduos selecionados
        offspring = list(map(toolbox.clone, offspring))
        # Aplica o crossover e a mutação
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < CXPB:
                toolbox.mate(child1, child2)
                del child1.fitness.values
                del child2.fitness.values
        for mutant in offspring:
            if random.random() < MUTPB:
                toolbox.mutate(mutant)
                del mutant.fitness.values
        # Avalia os indivíduos com fitness inválida
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit
        # Substitui a população pelos filhos da geração anterior
        pop[:] = offspring
        # Junta as fitness em uma lista única e exibe o estado
        fits = [ind.fitness.values[0] for ind in pop]
        length = len(pop)
        mean = sum(fits) / length
        sum2 = sum(x*x for x in fits)
        std = abs(sum2 / length - mean ** 2) ** 0.5
        print("  Min %s" % min(fits))
        print("  Max %s" % max(fits))
        print("  Avg %s" % mean)
        print("  Std %s" % std)
