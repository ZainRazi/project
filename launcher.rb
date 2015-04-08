# encoding: utf-8
require_relative 'carGrammar'

define_type :car do
  extends(:setup, :simple)

  population  20
  elites      0

  representation :grammar_derivation, length: 1000, max_wraps: 1, root: 'main', rules: grammar()

  selector :s, :tournament, size: 5
  variator :x, :one_point_crossover, rate: 0.7, source: :s
  variator :m, :uniform_mutation, rate: 0.2, source: :x



  # REMEMBER TO SET YOUR PATH USING THE PATH VARIABLE BELOW:
  # - Use an absolute path.
  evaluator :car, [threads: 1, path: "~/lookhere/UGV"] do

    return Fitness::Worst.new if car.nil?
    
    x = "#{car}"
    x.slice!("#<Chromosome::Car:")
    x.slice!(">")
 
    results = battle(car)

    return Fitness::Simple.new(true, results)

  end

  # The number of evaluations after which your algorithm should terminate.
  termination_condition :evaluations, :evaluations, limit: 50000

  logger :best_individual,    output_directory: "output"
  logger :full_fitness_dump,  output_directory: "output"

end

# Try changing the mode to :jruby to add multi-threading.
run(:car, mode: :ruby)
