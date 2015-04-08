# encoding: utf-8
import('../type/bootstrap')

define_type :best do
  extends(:fitness)
  composer { s(:fitness, :best) }
end

define_specification :best do
  extends(:fitness)

  # Compares this fitness to another.
  method :<=>, returns: ruby(:integer), accepts: [
    parameter(:other, s(:fitness))
  ] do
    return other.class == Fitness::Best ? 0 : 1
  end

  # Produces a string description of this fitness.
  method :to_s, returns: ruby(:string) do
    return "BEST"
  end
end
