# encoding: utf-8
require_relative 'carGrammar'

define_type :car do
  extends(:setup, :simple)

  population  10
  elites      2

  representation :grammar_derivation, length: 500000, max_wraps: 2, root: 'main', rules: grammar()

  selector :s, :tournament, size: 3
  variator :x, :two_point_crossover, rate: 0.7, source: :s
  variator :m, :uniform_mutation, rate: 0.6, source: :x



  # REMEMBER TO SET YOUR PATH USING THE PATH VARIABLE BELOW:
  # - Use an absolute path.
  evaluator :car, [threads: 1, path: "~/lookhere/UGV"] do

    return Fitness::Worst.new if car.nil?
    
    x = "#{car}"
    x.slice!("#<Chromosome::Car:")
    x.slice!(">")
 
    results = 0
    
    i = 0
    
    while (i < 5) do
        results = results + battle(car)
        i = i + 1
    end

    return Fitness::Simple.new(true, results)

  end

  # The number of evaluations after which your algorithm should terminate.
  termination_condition :evaluations, :evaluations, limit: 50000

  logger :best_individual,    output_directory: "output"
  logger :full_fitness_dump,  output_directory: "output"

end

# Try changing the mode to :jruby to add multi-threading.
run(:car, mode: :ruby)
