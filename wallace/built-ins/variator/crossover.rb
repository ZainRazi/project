# encoding: utf-8
import('../type/variator')

register t(:crossover) {
  extends(:variator)
  composer { s(:variator, :crossover) }
}

register s(:crossover) {
  extends(:variator)

  attribute :rate, ruby(:float)

  constructor [
    parameter(:name,        ruby(:string)),
    parameter(:source,      ruby(:*)),
    parameter(:num_inputs,  ruby(:string)),
    parameter(:rate,        ruby(:float))
  ] do
    super(name, source, num_inputs)
    self.rate = rate
  end

  method :crossover?, returns: ruby(:boolean), accepts: [
    parameter(:rng, s(:rng))
  ] do
    rng.float(0.0, 1.0) <= rate
  end

}
