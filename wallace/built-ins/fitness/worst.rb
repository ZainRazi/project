# encoding: utf-8
import('../type/bootstrap')

define_type :worst do
  extends(:fitness)
  composer { s(:fitness, :worst) }
end

define_specification :worst do
  extends(:fitness)

  # Compares this fitness to another.
  method :<=>, returns: ruby(:integer), accepts: [
    parameter(:other, s(:fitness))
  ] do
    return other.class == Fitness::Worst ? 0 : -1
  end

  # Produces a string description of this fitness.
  method :to_s, returns: ruby(:string) do
    return "WORST"
  end
end
