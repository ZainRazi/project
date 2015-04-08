# encoding: utf-8
import('../crossover')

define_type :uniform_crossover do
  extends(:variator, :crossover)
  composer { s(:variator, :uniform_crossover) }
end

define_specification :uniform_crossover do
  extends(:variator, :crossover)

  attribute :parent_bias, ruby(:float)

  # Constructs a new uniform crossover variator.
  constructor [
    parameter(:name,        ruby(:string)),
    parameter(:source,      ruby(:*)),
    parameter(:rate,        ruby(:float)),
    parameter(:parent_bias, ruby(:float), 0.5)
  ] do
    self.parent_bias = parent_bias
    super(name, source, 2, rate)
  end

  implement_method :operate do
    for i in 0 ... inputs[0].length
      if crossover?(rng)
        inputs[0][i] = rng.float() <= parent_bias ? inputs[0][i] : inputs[1][i]
      end
    end
    return [inputs[0]]
  end

end
