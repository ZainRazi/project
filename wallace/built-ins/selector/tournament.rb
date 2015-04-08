# encoding: utf-8
import('../type/selector')

define_specification :tournament do
  extends(:selector)

  attribute :size,      ruby(:integer)
  attribute :pick_best, ruby(:boolean)

  constructor [
    parameter(:name,      ruby(:string)),
    parameter(:size,      ruby(:integer), 7),
    parameter(:pick_best, ruby(:boolean), true)
  ] do
    super(name)
    self.size = size
    self.pick_best = pick_best
  end

  implement_method :prepare do
    return candidates
  end

  implement_method :select do
    Array.new(num_requested) do
      t = rng.sample(candidates, size)
      pick_best ? t.max : t.min
    end
  end

end

define_type :tournament do
  extends(:selector)
  composer do
    s(:selector, :tournament)
  end
end
