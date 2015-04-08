# encoding: utf-8
# define_type :koza_tree do
#   extends(:representation)

#   indexed_collection  :functions, :function, ruby(:proc)
#   indexed_collection  :ephemerals, :ephemeral, i(:ephemeral)

#   attribute :literals, array(ruby(:string))

# end

# representation :koza_tree do

#   max_depth 30
#   min_depth 5

#   function :add, { |x, y| x + y }
#   function :sub, { |x, y| x - y }
#   function :mul, { |x, y| x * y }

#   ephemeral type: :float, min: 0.0, max: 1.0

#   symbol  :x
#   symbol  :y

#   symbols [:x, :y, :z]

# end
