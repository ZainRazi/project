# encoding: utf-8
import('../crossover')

define_type :two_point_crossover do
  extends(:variator, :crossover)
  composer { s(:variator, :two_point_crossover) }

end

define_specification :two_point_crossover do
  extends(:variator, :crossover)

  # Constructs a new two-point crossover variator.
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

    # Ensure that the individuals are longer than length 2!
    return inputs if inputs[0].length <= 2 or inputs[1].length <= 2

    # Calculate the crossover points X and Y then swap the substrings
    # between A and B at those loci.
    x = rng.integer(1, [inputs[0].length, inputs[1].length].min - 1, false)
    y = rng.integer(x, [inputs[0].length, inputs[1].length].min, false)

    # Unfortunately []= can not be performed in parallel so we must
    # find and store a substring before swapping them.
    t = inputs[0].slice(x, y)
    inputs[0].splice(x, y, inputs[1].slice(x, y))
    inputs[1].splice(x, y, t)

    return inputs

  end

end
