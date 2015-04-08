# encoding: utf-8
import('../../type/variator')

define_type :gaussian_mutation do
  extends(:variator)
  composer { s(:variator, :gaussian_mutation) }
end

define_specification :gaussian_mutation do
  extends(:variator)

  # The per-bit probability of a mutation being performed.
  attribute :rate,  ruby(:float)

  # The mean of the distribution.
  attribute :mean,  ruby(:float)

  # The standard deviation of the distribution.
  attribute :std,   ruby(:float)

  # Constructs a new gaussian mutation operator.
  constructor [
    parameter(:name,    ruby(:string)),
    parameter(:rate,    ruby(:float)),
    parameter(:mean,    ruby(:float)),
    parameter(:std,     ruby(:float)),
    parameter(:source,  ruby(:*))
  ] do
    super(name, source, 1)
    self.rate = rate
    self.mean = mean
    self.std = std
  end

  implement_method :operate do
    (0 ... inputs[0].length).each { |i|
      inputs[0][i] = inputs[0][i] + rng.gaussian(mean, std) if rng.float <= rate
    }
    return inputs
  end

end
