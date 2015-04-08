# encoding: utf-8
import('../type/criterion')

register t(:evaluations) {
  extends(:criterion)

  satisfied? do
    statistics.evaluations >= limit
  end

  composer do
    s = sup
    s.attribute(:limit, ruby(:integer))
    s.constructor([
      s.parameter(:limit, ruby(:integer))
    ], source: "self.limit = limit")
    s
  end

}
