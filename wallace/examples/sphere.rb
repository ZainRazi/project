# encoding: utf-8

# Define setup for sphere problem.
define_type :sphere do
  extends(:setup, :simple)

  # Here we can specify the size of the population and the number of elites.
  population  500
  elites      1

  # Vector of real numbers.
  representation :real_vector, length: 20, min: -5.12, max: 5.12

  # Breeding operators.
  selector :s, :tournament, size: 5
  variator :x, :uniform_crossover, rate: 0.7, source: :s
  variator :m, :gaussian_mutation, rate: 0.05, mean: 0.0, std: 0.5, source: :x

  # Sphere function.
  evaluator :simple, [threads: 8] do
    return Fitness::Simple.new(false, chromosome.contents.inject(0.0) { |sum, x| sum + x**2 })
  end

  # The algorithm will terminate whenever any of these conditions are satisfied.
  termination_condition :iterations, :iterations, limit: 1000
  #termination_condition :evaluations, :evaluations, limit: 5000

end

# Runs this given setup (in a sub-task).
run(:sphere, mode: :ruby)
