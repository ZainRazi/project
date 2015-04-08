# encoding: utf-8
import('../crossover')

define_type :one_point_crossover do
  extends(:variator, :crossover)
  composer { s(:variator, :one_point_crossover) }

end

define_specification :one_point_crossover do
  extends(:variator, :crossover)

  constructor [
    parameter(:name, ruby(:string)),
    parameter(:rate, ruby(:float)),
    parameter(:source, ruby(:*))
  ] do
    super(name, source, 2, rate)
  end

  implement_method :operate do

    # Enforce the crossover rate and that the individuals are longer than length 1!
    return inputs unless crossover?(rng)

    # Ensure that the individuals are longer than length 1!
    return inputs if inputs[0].length <= 1 or inputs[1].length <= 1

    # Calculate the crossover point, split A and B into four substrings
    # and combine those substrings to form two children. 
    x = rng.integer(1, [inputs[0].length, inputs[1].length].min)

    # Generate the output individuals.
    return [
      inputs[0].slice(0, x).concat(inputs[1].slice(x, inputs[1].length)),
      inputs[1].slice(0, x).concat(inputs[0].slice(x, inputs[0].length))
    ]

  end

end
