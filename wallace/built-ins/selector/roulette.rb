# encoding: utf-8
import('../type/selector')

# register s(:roulette) {
#   extends(:selector)

#   attribute :size,      ruby(:integer)
#   attribute :pick_best, ruby(:boolean)

#   constructor [
#     parameter(:name,      ruby(:string)),
#     parameter(:size,      ruby(:integer), 7),
#     parameter(:pick_best, ruby(:boolean), true)
#   ] do
#     super(name)
#     self.size = size
#     self.pick_best = pick_best
#   end

#   implement_method :prepare do
#     return candidates
#   end

#   implement_method :select do
#     Array.new(num_requested) do
#       t = rng.sample(candidates, size)
#       pick_best ? t.min : t.max
#     end
#   end

# }

# register t(:roulette) {
#   extends(:selector)
#   composer { s(:selector, :roulette) }
# }
