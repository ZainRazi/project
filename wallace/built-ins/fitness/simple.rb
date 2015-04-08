# encoding: utf-8
import('../type/bootstrap')

define_type :simple do
  extends(:fitness)
  composer { s(:fitness, :simple) }
end

define_specification :simple do
  extends(:fitness)

  # A flag indicating whether the fitness should be maximised.
  attribute :maximise,  ruby(:boolean)

  # The single value of this fitness.
  attribute :value,     ruby(:float)

  # Constructs a new fitness object.
  constructor [
    parameter(:maximise,  ruby(:boolean)),
    parameter(:value,     ruby(:float))
  ] do
    self.maximise = maximise
    self.value = value
  end

  # Compares this fitness to another.
  method :<=>, returns: ruby(:integer), accepts: [
    parameter(:other, s(:fitness))
  ] do
    if other.class == Fitness::Simple
      return maximise ? (value <=> other.value) : (other.value <=> value)
    else
      super(other)
    end
  end

  # Produces a string description of this fitness.
  method :to_s, returns: ruby(:string) do
    return value.to_s
  end

end
